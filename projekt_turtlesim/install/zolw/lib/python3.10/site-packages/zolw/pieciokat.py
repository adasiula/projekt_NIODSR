#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math

PI = 3.1415926535897

class DrawPentagonNode(Node):

    #niezmienialne pozycja
    poz_x=0.0
    poz_y=0.0
    poz_x0=0.0
    poz_y0=0.0
    poz_x1=0.0
    poz_y1=0.0
    poz_x2=0.0
    poz_y2=0.0
    poz_x3=0.0
    poz_y3=0.0
    poz_x4=0.0
    poz_y4=0.0
    x_goal=0.0
    y_goal=0.0
    w=0

    #niezmienialne obrot
    kat_theta=0.0

    #flagi do rysowania i obracania
    pobranie_pozycji_pieciokata=True
    wierzcholek=False
    rysowanie_boku=False
    obracanie=False
    
    #parametry do rysowania bokow
    dystans=2.0

    #parametry do obracania
    szb_obr=45.0 # stopnie/s
    kat=108 # stopnie 
    kat_obrotu=180 - kat 


    #niezmienialne konwersje
    katowa_prd = szb_obr*2*PI/360
    rel_kat = kat_obrotu*2*PI/360
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
        self.kat_theta=pose.theta
        # print('x y theta')
        # print(self.poz_x)
        # print(self.poz_y)
        # print(self.kat_theta)
        
        velocity_message = Twist()

        #pobranie wspolrzednych pieciokata

        if(self.pobranie_pozycji_pieciokata):
            print('pobranie pozycji')
            self.poz_x0=self.poz_x
            self.poz_y0=self.poz_y
            self.poz_x1=self.poz_x0+math.cos(pose.theta)*self.dystans
            self.poz_y1=self.poz_y0+math.sin(pose.theta)*self.dystans
            self.poz_x2=self.poz_x1+math.cos(pose.theta+self.rel_kat)*self.dystans
            self.poz_y2=self.poz_y1+math.sin(pose.theta+self.rel_kat)*self.dystans
            self.poz_x3=self.poz_x2+math.cos(pose.theta+2*self.rel_kat)*self.dystans
            self.poz_y3=self.poz_y2+math.sin(pose.theta+2*self.rel_kat)*self.dystans
            self.poz_x4=self.poz_x3+math.cos(pose.theta+3*self.rel_kat)*self.dystans
            self.poz_y4=self.poz_y3+math.sin(pose.theta+3*self.rel_kat)*self.dystans
            
            self.pobranie_pozycji_pieciokata=False
            self.wierzcholek=True
            print('rysowanie pięciokąta')

        #wierzchołek
        if(self.wierzcholek):
            
            match self.w:
                case 0:
                    self.x_goal=self.poz_x1
                    self.y_goal=self.poz_y1
                case 1:
                    self.x_goal=self.poz_x2
                    self.y_goal=self.poz_y2
                case 2:
                    self.x_goal=self.poz_x3
                    self.y_goal=self.poz_y3
                case 3:
                    self.x_goal=self.poz_x4
                    self.y_goal=self.poz_y4
                case 4:
                    self.x_goal=self.poz_x0
                    self.y_goal=self.poz_y0

            self.w=self.w +1

            if self.w > 4:
                self.w = 0
            
            self.rysowanie_boku=True
            self.wierzcholek=False
            

        #rysowanie boku
            
        if(self.rysowanie_boku):
            K_linear = 2.0 #0.5
            dis = abs(math.sqrt((((self.x_goal - pose.x) ** 2) + ((self.y_goal - pose.y) ** 2))))

            linear_speed = dis * K_linear

            K_angular = 4.0
            desired_angle_goal = math.atan2( self.y_goal - pose.y, self.x_goal-pose.x)
            angular_speed = (desired_angle_goal - pose.theta) * K_angular

            if(angular_speed>5.0):
                angular_speed=5.0
            if(angular_speed<-5.0):
                angular_speed=-5.0
            
            velocity_message.linear.x = linear_speed
            velocity_message.angular.z = angular_speed

            self.cmv_vel_pub_.publish(velocity_message)
            if (dis <= 0.01):
                velocity_message.angular.z = 0.0
                self.obracanie=True
                self.rysowanie_boku=False

        #obracanie

        if(self.obracanie):
            velocity_message.linear.x=0.0
            velocity_message.linear.y=0.0
            velocity_message.linear.z=0.0
            velocity_message.angular.x = 0.0
            velocity_message.angular.y = 0.0
            velocity_message.angular.z=self.katowa_prd

            obecny_kat=0.0
            self.t0=time.time()
            while(obecny_kat<self.rel_kat):
                self.cmv_vel_pub_.publish(velocity_message)
                self.t1 = time.time()
                obecny_kat=self.katowa_prd*(self.t1-self.t0)
            
            velocity_message.angular.z = 0.0
            self.cmv_vel_pub_.publish(velocity_message)
            self.wierzcholek=True
            self.rysowanie_boku=True
            self.obracanie=False


def main(args=None):
    rclpy.init(args=args)
    node_pentagon = DrawPentagonNode()
    node_pentagon.dystans=float(input('dlugosc boku?\n'))
    rclpy.spin(node_pentagon)
    rclpy.shutdown()