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

# Parameters
k = 0.1  # look forward gain
Lfc = 0.75  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.04  # [s] time tick
WB = 0.28  # [m] wheel base of vehicle

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

class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_f1_tenth_controller')
        
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
                
        self.target_speed = 3.5 # [units/s]
        
        self.target_path = TargetPath(x_r, y_r)
        self.target_idx = 28
        self.search_target_samples = 10

        self.pose_subscription_ = self.create_subscription(PoseStamped, '/ground_truth/pose', 
                                                           self.pure_pursuite_controll, 10)
        
    def pure_pursuite_controll(self, pose: PoseStamped):
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
        
        # rank_target = []
        # for idx in range(int(-self.search_target_samples/2), int(self.search_target_samples/2)):
        #     n_target = self.target_idx + idx
        #     if n_target < 0:
        #         n_target = self.target_path.count + n_target
        #     n_target = n_target % self.target_path.count
        #     dst = self.target_path.states[n_target].calc_distance(pose_l[0], pose_l[1])
        # rank_target

        current_state = State(x=pose_l[0], y=pose_l[1], yaw=yaw_from_quaternion(pose_l[3:]), v=self.state.v)
        self.state = current_state
        ai = self.proportional_control()
        di = self.pure_pursuit_steer_control(current_state, dst)
        ai = np.clip(ai, 0.005, 2.0)
        self.state.a = ai
        
        self.state.update(ai, di)
        
        self.publish_control(di, ai)
        
    def proportional_control(self):
        a = Kp * (self.target_speed - self.state.v)

        return a

    def pure_pursuit_steer_control(self, state: State, Lf):
        
        if Lf == 0:
            return 0.0
        
        target_state = self.target_path.states[self.target_idx]

        alpha = math.atan2(target_state.y - state.rear_y, target_state.x - state.rear_x) - state.yaw

        delta = math.atan2(2.0 * WB * math.sin(alpha), Lf)

        return delta

    def publish_control(self, steer, accel):
        acc_msg = AckermannControlCommand()
        acc_msg.lateral.steering_tire_angle = steer
        acc_msg.longitudinal.acceleration = accel
        acc_msg.longitudinal.speed = self.target_speed
        self.publisher_.publish(acc_msg)
        self.get_logger().info(f'Published acc: {accel} and steer: {steer}')
        

def main(args=None):
    rclpy.init(args=args)

    pure_pursuit_controller = PurePursuitController()

    rclpy.spin(pure_pursuit_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pure_pursuit_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()