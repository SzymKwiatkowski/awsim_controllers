import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSReliabilityPolicy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from geometry_msgs.msg import PoseStamped

import math
import numpy as np
import pandas as pd
import bisect

# Parameters
k = 0.1  # look forward gain
Lfc = 0.75  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.04  # [s] time tick
WB = 0.28  # [m] wheel base of vehicle

class CubicSpline1D:
    def __init__(self, x, y):

        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return position

    def calc_first_derivative(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy

    def calc_second_derivative(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def __search_index(self, x):
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]\
                - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B

class CubicSpline2D:
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)

        return x, y

    def calc_curvature(self, s):
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw

def calc_spline_course(x, y, ds=0.1):
    sp = CubicSpline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, a=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.a = a
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
    
    def calc_distance(self, x, y):
        return math.sqrt((self.x-x)**2 + (self.y-y)**2)

class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)

class TargetPath:

    def __init__(self, cx, cy):
        self.states = [State(el, cy[idx]) for idx, el in enumerate(cx)]
        self.count = len(cx)
        
    def next_idx(self, current_idx):
        if current_idx+1 >= self.count:
            return 0
        
        return current_idx+1

def yaw_from_quaternion(q):
    return np.arctan2(2.0*(q[0]*q[1] + q[3]*q[2]), q[3]**2 + q[0]**2 - q[1]**2 - q[2]**2)

class StanleyController(Node):

    def __init__(self):
        super().__init__('stanley_controller')
        
        options = QoSProfile(depth=1)
        options.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        options.reliability = QoSReliabilityPolicy.RELIABLE
        self.publisher_ = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', qos_profile=options)
        
        self.declare_parameter('waypoints_file', 'config/waypoints.csv')
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        df_waypoints = pd.read_csv(waypoints_file)
        self.thresh_err = 0.1
        initial_state = [2.28139033424668e-05,
                         -6.034809985067113e-07,
                         -6.034809985067113e-07,
                         3.259629011154175e-09,
                         4.117931530345231e-05,
                         -9.948154911398888e-06,
                         1.0]
        self.time_nanosecs = self.get_clock().now().nanoseconds
        self.state = State(x=initial_state[0], y=initial_state[1], yaw=yaw_from_quaternion(initial_state[3:]), v=0.0)
        x_r = df_waypoints['pose.x'].to_numpy()
        y_r = df_waypoints['pose.y'].to_numpy()
        cx, cy, cyaw, ck, s = calc_spline_course(
            x_r, y_r, ds=0.1)
        self.cyaw = cyaw     
        self.target_speed = 3.5 # [units/s]
        
        self.target_path = TargetPath(cx, cy)
        self.last_target_idx = 27
        self.target_idx = 28
        self.search_target_samples = 10

        self.pose_subscription_ = self.create_subscription(PoseStamped, '/ground_truth/pose', 
                                                           self.stanley_controll, 10)
        
    def stanley_controll(self, pose: PoseStamped):
        pose_l = [pose.pose.position.x, 
                 pose.pose.position.y,
                 pose.pose.position.z, 
                 pose.pose.orientation.x,
                 pose.pose.orientation.y, 
                 pose.pose.orientation.z, 
                 pose.pose.orientation.w]
        
        
        dst = self.target_path.states[self.target_idx].calc_distance(pose_l[0], pose_l[1])
        Lf = Lfc + k * self.state.v
        while dst < Lf:
            self.target_idx = self.target_path.next_idx(self.target_idx)
            dst = self.target_path.states[self.target_idx].calc_distance(pose_l[0], pose_l[1])
            self.get_logger().info(f"err: {dst}")

        current_state = State(x=pose_l[0], y=pose_l[1], yaw=yaw_from_quaternion(pose_l[3:]), v=self.state.v)
        self.state = current_state
        ai = self.proportional_control()
        di = self.stanley_steer_control()
        ai = np.clip(ai, 0.005, 2.0)
        self.state.a = ai
        
        self.state.update(ai, di)
        
        self.publish_control(di, ai)
        
    def proportional_control(self):
        a = Kp * (self.target_speed - self.state.v)

        return a

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def stanley_steer_control(self):
        # Project RMS error onto front axle vector
        target_state = self.target_path.states[self.target_idx]
        front_axle_vec = [-np.cos(self.state.yaw + np.pi / 2),
                          -np.sin(self.state.yaw + np.pi / 2)]
        error_front_axle = np.dot([target_state.x, target_state.y], front_axle_vec)

        if self.last_target_idx >= current_target_idx:
            current_target_idx = self.last_target_idx

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(self.cyaw[self.tar] - self.state.yaw)
        # theta_d corrects the cross track error
        theta_d = np.arctan2(k * error_front_axle, self.state.v)
        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx

    def publish_control(self, steer, accel):
        acc_msg = AckermannControlCommand()
        acc_msg.lateral.steering_tire_angle = steer
        acc_msg.longitudinal.acceleration = accel
        acc_msg.longitudinal.speed = self.target_speed
        self.publisher_.publish(acc_msg)
        self.get_logger().info(f'Published acc: {accel} and steer: {steer}')
        

def main(args=None):
    rclpy.init(args=args)

    stanley_controller = StanleyController()

    rclpy.spin(stanley_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stanley_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()