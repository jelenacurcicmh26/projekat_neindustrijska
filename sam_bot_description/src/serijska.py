#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np


def serijski_parser(received_data): #poruka tipa 0xFF, 0xFF, id, val1, val2, val3, val4, val5, val6, val7, checksum
    data_list = received_data.split(b',')

    if len(data_list) != 12:
        return
    
    if data_list[1] and data_list[2] != 0xFF:
        return
    
    message_id = int(data_list[2], 16)
    val1 = int(data_list[3], 16)
    val2 = int(data_list[4], 16)
    val3 = int(data_list[5], 16)
    val4 = int(data_list[6], 16)
    val5 = int(data_list[7], 16)
    val6 = int(data_list[8], 16)
    val7 = int(data_list[9], 16)
    checksum = int(data_list[10], 16)

    calculated_checksum = (~(message_id + val1 + val2 + val3 + val4+ val5 + val6 + val7)) & 0xFF

    if checksum != calculated_checksum:
        return
    
    if message_id == 1:
        return 1, odometrija_pub(val1, val2, val3, val4, val5, val6, val7) #x1, x2, y1, y2, ugao, linearna brzina, ugaona brzina
    elif message_id == 2:
        detektovano(val1, val2, val3) #recimo da se salju snimanja tri senzora
        return 2, None

def odometrija_pub(x1, x2, y1, y2, ugao, brzina_x, brzina_y):
    x = (x1 << 8) | x2
    y = (y1 << 8) | y2

    x = x / 1000.0 #zato sto ce se slati u milimetrima
    y = y / 1000.0

    brzina_x = brzina_x / 1000.0 #ako se salje u mm/s, da bi bilo m/s
    brzina_y = brzina_y / 1000.0

    roll = 0 
    pitch = 0 
    yaw = ugao #yaw = math.radians(ugao) AKO SE SALJU STEPENI A NE RADIJANI

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2)

    odom_msg = Odometry()
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_link'
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation.w = qw  # Set the quaternion w component
    odom_msg.pose.pose.orientation.x = qx  # Set the quaternion x component
    odom_msg.pose.pose.orientation.y = qy  # Set the quaternion y component
    odom_msg.pose.pose.orientation.z = qz  # Set the quaternion z component
    odom_msg.twist.twist.linear.x = brzina_x  # Set the linear velocity along x-axis
    odom_msg.twist.twist.linear.y = brzina_y  # Set the linear velocity along y-axis
    odom_msg.twist.twist.angular.z = 0.0  # Set the angular velocity around z-axis

    odom_msg.covariance[0] = 0.01  # Kovarijansa za x
    odom_msg.covariance[7] = 0.01  # Kovarijansa za y
    odom_msg.covariance[14] = 0.00  # Kovarijansa za z

    return odom_msg

def detektovano(senzor1, senzor2, senzor3): #DODATI STA RADITI U OVOM SLUCAJU (tipa da se ocisti kostmapa, da se prikazu novi laserscanovi, itd)
    if senzor1 and not senzor2 and not senzor3:
        print('Senzor 1 je nesto detektovao a ti vidi sta ces')
    elif senzor2 and not senzor1 and not senzor3:
        print('Senzor 2 je nesto detektovao a ti vidi sta ces')
    elif senzor3 and not senzor2 and not senzor1:
        print('Senzor 3 je nesto detektovao a ti vidi sta ces')
    else:
        print('Vise senzora je nesto ovde detektovalo')

def cmd_vel_callback(msg, serial_connection):
    linearna = int(msg.linear.x * 100.0) 
    ugaona = int(msg.angular.z * 100.0)   

    checksum = (~(linearna + ugaona)) & 0xFF

    poruka = bytes.fromhex(f'FF FF {linearna:02X} {ugaona:02X} {checksum:02X}')

    serial_connection.write(poruka)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('serial_node')
    
    serial_port = '/dev/ttyUSB0' 
    baud_rate = 9600  
    serial_connection = serial.Serial(serial_port, baud_rate)

    odom_publisher = node.create_publisher(Odometry, '/odom', 10)
    
    cmd_vel_subscriber = node.create_subscription(Twist, '/cmd_vel', lambda msg: cmd_vel_callback(msg, serial_connection), 10)

    while rclpy.ok():  
        received_data = serial_connection.readline().strip() #CEKA /n DA BI ZAVRSIO JEDNO CITANJE
        if received_data: 
            id, odom_msg = serijski_parser(received_data)
            if id == 1:
                 odom_publisher.publish(odom_msg)
            elif id == 2:
                print('Usao sam u else')
            else:
                print('Nije pronadjen id')
            
            
    serial_connection.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()