#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math
#import rospy

PI= 3.14
acurate_PI = 3.1415926535897
max_obrot=3.1317784786224365
min_obrot=-3.141592502593994
#3.1290547847747803
najwiekszy=3.1290547847747803
#-3.135406970977783
najmniejszy=-3.135406970977783



class DrawSquareNode(Node):

    #niezmienialne pozycja
    poz_x=0.0
    poz_y=0.0
    poz_x0=0.0
    poz_y0=0.0
    distacne_moved = 0.0

    #niezmienialne obrot
    obrot=0.0
    obr=0.0
    error=0.0225

    #flagi do rysowania i obracania
    koniec_boku=False
    poczatek_boku=False
    koniec_obrotu=True
    poczatek_obrotu=False

    #parametry do rysowania bokow
    speed=1.0
    distance=4.0
    is_forward=True

    #parametry do obracania
    kat=(abs(najmniejszy)+najwiekszy)/4
    szybkosc_obrotu=0.3 #(degrees/s)

    def __init__(self):
        super().__init__("draw_square")
        self.cmv_vel_pub_=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.poz_sub=self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
        self.get_logger().info("Draw square node has been started")

    def pose_callback(self,pose:Pose):

        self.poz_x=pose.x
        self.poz_y=pose.y
        self.obr=pose.theta
        print('x y theta')
        print(self.poz_x)
        print(self.poz_y)
        print(self.obr)
        

        velocity_message = Twist()
        print("x")
        print(pose.x)
        print("y")
        print(pose.y)

        #początek rysowania boku

        if not(self.poczatek_boku):
            self.poz_x0=self.poz_x
            self.poz_y0=self.poz_y
            self.poczatek_boku=True

        #końcowy kąt theta

        if not(self.poczatek_obrotu):
            self.obrot=pose.theta + self.kat-self.error
            if(self.obrot>najwiekszy):
                self.obrot=-abs(abs(najmniejszy) - abs(najwiekszy-self.obrot))
            
            self.poczatek_obrotu=True
        
        #rysowanie boku
            
        if not(self.koniec_boku):
            print('ruch')
            if(self.is_forward):
                velocity_message.linear.x=abs(self.speed)
            else:
                velocity_message.linear.x=-abs(self.speed)

            self.cmv_vel_pub_.publish(velocity_message)

            time.sleep(0.2)
            
            print(self.poz_x0)
            self.distacne_moved = self.distacne_moved+abs(0.5 * math.sqrt(((pose.x-self.poz_x0)**2)+((pose.y-self.poz_y0)**2)))
            
            print('distance moved')
            print(self.distacne_moved)
            
            if(self.distacne_moved>self.distance):
                velocity_message.linear.x=0.0
                print('koncowy x')
                print(pose.x)
                self.koniec_boku=True
                self.distacne_moved=0.0
                self.koniec_obrotu=False

            print('koniec ruchu')
            self.cmv_vel_pub_.publish(velocity_message)

        #obracanie

        if not(self.koniec_obrotu):
            print('obrot')
            print('theta')
            print(pose.theta)
            print('kat koncowy obrotu')
            print(self.obrot)

            velocity_message.angular.z = self.szybkosc_obrotu
            if(self.obrot>0):
                if(self.obrot<=pose.theta):
                    self.error=(abs(self.obrot-pose.theta)+self.error)/2
                    self.koniec_obrotu=True
                    self.koniec_boku=False
                    self.poczatek_obrotu=False
                    self.poczatek_boku=False
                    velocity_message.angular.z = 0.0
            if(self.obrot<0):
                if(pose.theta>0):
                    pass
                if(pose.theta<=0):
                    if(pose.theta>=self.obrot):
                        self.error=(abs(self.obrot-pose.theta)+self.error)/2
                        self.koniec_obrotu=True
                        self.koniec_boku=False
                        self.poczatek_obrotu=False
                        self.poczatek_boku=False
                        velocity_message.angular.z = 0.0

            self.cmv_vel_pub_.publish(velocity_message)


def main(args=None):
    rclpy.init(args=args)
    node_square = DrawSquareNode()
    rclpy.spin(node_square)
    rclpy.shutdown()