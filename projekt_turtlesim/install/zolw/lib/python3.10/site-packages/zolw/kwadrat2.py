#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math
#import rospy

PI = 3.1415926535897

class DrawSquareNode(Node):

    #niezmienialne pozycja
    poz_x=0.0
    poz_y=0.0
    poz_x0=0.0
    poz_y0=0.0
    poz_x1=0.0
    poz_y1=0.0
    x_goal=0.0
    y_goal=0.0
    w=0
    distacne_moved = 0.0

    #niezmienialne obrot
    obrot=0.0
    obr=0.0
    error=0.0225

    #flagi do rysowania i obracania
    pobranie_pozycji_kwadratu=True
    wierzcholek=False
    rysowanie_boku=False
    obracanie=False
    
    #parametry do rysowania bokow
    distance=4.0
    is_forward=True

    #parametry do obracania
    szb_obr=15.0 #(degrees/s)
    kat=90 #degrees

    #niezmienialne konwersje
    katowa_prd = szb_obr*2*PI/360
    rel_kat = kat*2*PI/360
    t0=0.0
    t1=0.0

    def __init__(self):
        super().__init__("draw_square")
        self.cmv_vel_pub_=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.poz_sub=self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
        self.get_logger().info("Draw square node has been started")

    def pose_callback(self,pose:Pose):

        self.poz_x=pose.x
        self.poz_y=pose.y
        self.obr=pose.theta
        # print('x y theta')
        # print(self.poz_x)
        # print(self.poz_y)
        # print(self.obr)
        

        velocity_message = Twist()

        #pobranie wspolrzednych kwadratu

        if(self.pobranie_pozycji_kwadratu):
            print('pobranie pozycji')
            self.poz_x0=self.poz_x
            self.poz_y0=self.poz_y
            self.poz_x1=self.poz_x0+self.distance
            self.poz_y1=self.poz_y0+self.distance
            self.pobranie_pozycji_kwadratu=False
            self.wierzcholek=True

        #wierzchołek
        if(self.wierzcholek):
            print('pobranie wierzcholka')
            match self.w:
                case 0:
                    self.x_goal=self.poz_x1
                    self.y_goal=self.poz_y0
                case 1:
                    self.x_goal=self.poz_x1
                    self.y_goal=self.poz_y1
                case 2:
                    self.x_goal=self.poz_x0
                    self.y_goal=self.poz_y1
                case 3:
                    self.x_goal=self.poz_x0
                    self.y_goal=self.poz_y0

            self.w=self.w +1

            if self.w > 3:
                self.w = 0
            
            self.rysowanie_boku=True
            self.wierzcholek=False
            

        #rysowanie boku
            
        if(self.rysowanie_boku):
            print('rysowanie boku')
            K_linear = 0.5
            dis = abs(math.sqrt((((self.x_goal - pose.x) ** 2) + ((self.y_goal - pose.y) ** 2))))

            linear_speed = dis * K_linear

            K_angular = 4.0
            desired_angle_goal = math.atan2( self.y_goal - pose.y, self.x_goal-pose.x)
            angular_speed = (desired_angle_goal - pose.theta) #* K_angular
            print('ang speed')
            print(angular_speed)
            velocity_message.linear.x = linear_speed
            velocity_message.angular.z = angular_speed

            self.cmv_vel_pub_.publish(velocity_message)
            if (dis <= 0.01):
                print('koniec rysowania')
                velocity_message.angular.z = 0.0
                self.obracanie=True
                self.rysowanie_boku=False

        #obracanie

        if(self.obracanie):
            print('obracanie')
            velocity_message.linear.x=0.0
            velocity_message.linear.y=0.0
            velocity_message.linear.z=0.0
            velocity_message.angular.x = 0.0
            velocity_message.angular.y = 0.0
            velocity_message.angular.z=self.katowa_prd

            obecny_kat=0.0
            self.t0=time.time()
            while(obecny_kat<self.rel_kat):
                print('obracanie 2')
                self.cmv_vel_pub_.publish(velocity_message)
                self.t1 = time.time()
                obecny_kat=self.katowa_prd*(self.t1-self.t0)
            
            velocity_message.angular.z = 0.0
            print('koniec obracania')
            self.cmv_vel_pub_.publish(velocity_message)
            self.wierzcholek=True
            self.rysowanie_boku=True
            self.obracanie=False


def main(args=None):
    rclpy.init(args=args)
    node_square = DrawSquareNode()
    node_square.distance=2.0
    rclpy.spin(node_square)
    rclpy.shutdown()