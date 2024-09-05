#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
import math

class JointPublisher(Node):
    def __init__(self):
        super().__init__('JointPublisher_Node')

        self.group = rclpy.callback_groups.ReentrantCallbackGroup()
        
        
        ##### PUBLISHER ######
        command_topics = [
            'LAposition_trajectory_controller/joint_trajectory',
            'RAposition_trajectory_controller/joint_trajectory',
            'RFposition_trajectory_controller/joint_trajectory',
            'LFposition_trajectory_controller/joint_trajectory',
            'LRposition_trajectory_controller/joint_trajectory',
            'RRposition_trajectory_controller/joint_trajectory',
        ]

        self.joints_pub = [
            self.create_publisher(JointTrajectory, topic, 10) for topic in command_topics
        ]        
        ##### PUBLISHER #####
        
        ##### SUBSCRIBER #####
        # self.create_subscription(Geometry, "moonbot_geometry", self.LEGcallback, 1)
        ##### SUBSCRIBER #####
        
        ##### TIMER #####
        self.timer_period = 0.2
        self.create_timer(self.timer_period, self.pub_callback)
        ##### TIMER #####

        self.target_received = False
        # joint names
        self.joint_names = {
            'LA': ["left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6"],
            'RA': ["right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6"],
            'RF': ["f_right_leg_joint1", "f_right_leg_joint2", "f_right_leg_joint3"],
            'LF': ["f_left_leg_joint1", "f_left_leg_joint2", "f_left_leg_joint3"],
            'LR': ["b_left_leg_joint1", "b_left_leg_joint2", "b_left_leg_joint3"],
            'RR': ["b_right_leg_joint1", "b_right_leg_joint2", "b_right_leg_joint3"],
        }
    

    def pub_callback(self):
        self.publish_joint_angles()

    def publish_joint_angles(self):
        # Target angle for each joint
        self.angles = {
            'LA': [math.pi, 1.047,-2.356, math.pi, 0.0, 0.0],
            'RA': [-math.pi, 1.047, -2.356, math.pi, 0.0, 0.0],
            'RF': [0.0, 0.0, 0.0],
            'LF': [0.0, 0.0, 0.0],
            'LR': [0.0, 0.0, 0.0],
            'RR': [0.0, 0.0, 0.0],
        }
        
        sec = self.timer_period
        
        for i, (limb, angles) in enumerate(self.angles.items()):
            msg = JointTrajectory()
            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=sec).to_msg()
            point.positions = angles

            msg.joint_names = self.joint_names[limb]
            msg.points = [point]

            self.joints_pub[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
