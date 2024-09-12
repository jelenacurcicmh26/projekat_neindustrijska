#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
import rclpy
import time as vreme
from rclpy.node import Node
from std_msgs.msg import String
import serial
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy import time
import numpy as np, math
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose



def zadavanje_pozicije(x, y, ugao, nav2_to_pose_client, jednom, node):
    if jednom:
        goal_pose = PoseStamped()

        roll = 0.0 
        pitch = 0.0 
        yaw = math.radians(ugao) #AKO SE SALJU STEPENI A NE RADIJANI

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)

        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        #goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        nav2_to_pose_client.send_goal(goal_pose)

        

# def zadavanje_pozicije(x, y, ugao, navigator, jednom):
#     #if jednom:
#         goal_pose = PoseStamped()

#         roll = 0.0 
#         pitch = 0.0 
#         yaw = math.radians(ugao) #AKO SE SALJU STEPENI A NE RADIJANI

#         qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#         qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#         qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#         qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)

#         goal_pose.header.frame_id = 'map'
#         goal_pose.header.stamp = navigator.get_clock().now().to_msg()
#         goal_pose.pose.position.x = x
#         goal_pose.pose.position.y = y
#         goal_pose.pose.orientation.x = qx
#         goal_pose.pose.orientation.y = qy
#         goal_pose.pose.orientation.z = qz
#         goal_pose.pose.orientation.w = qw

#         navigator.goToPose(goal_pose)

#         while not navigator.isTaskComplete():
#               print("Kretanje ka cilju!")

#         result = navigator.getResult()

def cinc(pocetak):
    if pocetak:
        pocetno = vreme.time()
        
    return pocetak, pocetno

def tajmer(pocetno):
    sistemsko_vreme = vreme.time() - pocetno
    return sistemsko_vreme
    
def main():
    rclpy.init()
    node = rclpy.create_node('task_publisher')
    #navigator = BasicNavigator()
    #goal_publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)
    task_publisher = node.create_publisher(Int8, '/task', 10)
    nav2_to_pose_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    pocetak = 1
    pocetno = 0
    korak = 1
    
    idemo, pocetno = cinc(1)
    jednom = 1
    
    while True:
        if idemo:
            sistemsko_vreme = tajmer(pocetno)
                 
            if korak == 1:
                zadavanje_pozicije(1.0, 1.0, 90.0, nav2_to_pose_client, jednom, node)
                jednom = 0
                korak = 2

        if sistemsko_vreme > 80:
            idemo = False

if __name__ == '__main__':
    pocetak = 1
    pocetno_vreme = 0
    korak = 1
         
    main()