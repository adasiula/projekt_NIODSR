#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

PI = 3.1415926535897


class DrawCircleNode(Node):

    promien=3.0
    predkosc_x = 3.0
    predkosc_obr_z = 3.0

    s=0.0 #s=a*(t**2)/2
    t=0.0
    a=0.0   #ad=(V**2)/r

    y_min=20.0
    y_max=0.0
    px=0.0
    py=0.0

    def __init__(self):
        super().__init__("draw_circle_p")
        self.cmv_vel_pub_=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.poz_sub=self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
        self.timer=self.create_timer(0.1,self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started p")

    def pose_callback(self,pose:Pose):
        self.px=pose.x
        self.py=pose.y

    def move(self):
        msg=Twist()
        
        self.predkosc_obr_z=self.predkosc_x/self.promien
        

        msg.linear.x=self.predkosc_x
        msg.angular.z=self.predkosc_obr_z
        self.cmv_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node_kolo = DrawCircleNode()
    node_kolo.promien=3.0
    node_kolo.predkosc_x=4.0
    rclpy.spin(node_kolo)
    rclpy.shutdown()