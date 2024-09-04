#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np
import math
from rclpy.action import ActionClient

class NavigateToPoseActionClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_action_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.result = None
        self.future = None

    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.future = self.action_client.send_goal_async(goal_msg)

    def get_result(self):
        if self.future is not None and self.future.done():
            self.result = self.future.result()
            return self.result
        else:
            return None

def zadavanje_pozicije(x, y, ugao):
    pose = PoseStamped()
    
    roll = 0.0 
    pitch = 0.0 
    yaw = math.radians(ugao)  # Convert degrees to radians

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)

    pose.header.frame_id = 'map'
    pose.header.stamp = rclpy.clock.Clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def main(args=None):
    rclpy.init(args=args)
    navigate_to_pose_client = NavigateToPoseActionClient()
    korak = 1
    
    while rclpy.ok():
        if korak == 1:
            goal_pose = zadavanje_pozicije(0.3, 0.3, 30.0)
            navigate_to_pose_client.send_goal(goal_pose)
            korak = 2

            if korak == 2:
                result = navigate_to_pose_client.get_result()

                print(result)
                
                if result and result.result == NavigateToPoseActionClient.Result.SUCCESS:
                    korak = 3

            rclpy.spin(navigate_to_pose_client)

    navigate_to_pose_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()