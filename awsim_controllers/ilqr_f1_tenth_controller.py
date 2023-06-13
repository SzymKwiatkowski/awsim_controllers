import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSReliabilityPolicy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Path

import math
import numpy as np
import pandas as pd


from .iLQR.cost import CostPath
from .iLQR.model import CarModel
from .iLQR.auto_derivatives import Derivatives
from .iLQR.iLQR import IterativeLQR

dt = 0.04  # [s] time tick
WB = 0.28  # [m] wheel base of vehicle


def yaw_from_quaternion(q):
    return np.arctan2(2.0*(q[0]*q[1] + q[3]*q[2]), q[3]**2 + q[0]**2 - q[1]**2 - q[2]**2)


class IterativeLQRController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_f1_tenth_controller')

        options = QoSProfile(depth=1)
        options.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        options.reliability = QoSReliabilityPolicy.RELIABLE
        self.publisher_ = self.create_publisher(
            AckermannControlCommand, '/control/command/control_cmd', qos_profile=options)
        self.ilqr_traj_pub = self.create_publisher(Path, '/ilqr_traj', 10)
        self.declare_parameter('waypoints_file', 'config/waypoints.csv')
        waypoints_file = self.get_parameter(
            'waypoints_file').get_parameter_value().string_value
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
        self.state = np.array([initial_state[0], initial_state[1], yaw_from_quaternion(
            initial_state[3:]), .0, .0])
        x_r = df_waypoints['pose.x'].to_numpy()
        y_r = df_waypoints['pose.y'].to_numpy()
        paths = np.vstack((x_r, y_r)).T
        paths = paths[0:555]
        model = CarModel(dt=dt)
        self.target_speed = 3.0  # [units/s]
        cost = CostPath(paths, velocity=self.target_speed, dt=dt, goal=250)
        derivs = Derivatives(model, cost)
        ilqr = IterativeLQR(model, cost, derivs)
        self.path = None
        N = 100 # paths.shape[0]
        max_iter = 50
        regu_init = 100
        self.u_trj = None
        x_trj, u_trj, cost_trace, regu_trace, redu_ratio_trace, redu_trace = ilqr.run_ilqr(
            self.state, N, max_iter, regu_init, u_init=None)

        self.u_trj = u_trj
        self.i = 0

        # for u in u_trj:
        #     self.publish_control(u[0], u[1])
        self.path = Path()
        for x in x_trj:
            state = PoseStamped()
            state.pose.position.x = float(x[0])
            state.pose.position.y = float(x[1])
            state.pose.position.z = 0.0
            q = quaternion_from_euler(0.0, 0.0, x[3])
            state.pose.orientation.x = q[0]
            state.pose.orientation.y = q[1]
            state.pose.orientation.z = q[2]
            state.pose.orientation.w = q[3]
            
            self.path.poses.append(state)
        self.get_logger().info("PUblished path")
        timer_period = 0.04  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pub_path = True
            

    def timer_callback(self):
        if self.pub_path and self.path:
            self.ilqr_traj_pub.publish(self.path)
            self.pub_path = False
            
        if self.u_trj is not None and self.i < len(self.u_trj):
            # acc = np.clip(self.u_trj[self.i, 0], 0.0, 1.0)
            acc = self.u_trj[self.i, 0]
            self.publish_control(acc, -self.u_trj[self.i, 1])
            self.i += 1

        
    def publish_control(self, accel, steer):
        acc_msg = AckermannControlCommand()
        acc_msg.lateral.steering_tire_angle = float(steer)
        acc_msg.longitudinal.acceleration = float(accel)
        acc_msg.longitudinal.speed = float(self.target_speed)
        self.publisher_.publish(acc_msg)
        self.get_logger().info(f'Published acc: {accel} and steer: {steer}')


def main(args=None):
    rclpy.init(args=args)

    ilqr_controller = IterativeLQRController()

    rclpy.spin(ilqr_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ilqr_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
