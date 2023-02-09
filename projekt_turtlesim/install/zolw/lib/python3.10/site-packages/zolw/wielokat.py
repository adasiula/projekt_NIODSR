#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math

PI = 3.1415926535897

class DrawPolygonNode(Node):

    #niezmienialne pozycja
    n=int(input('ile wierzcholkow?\n')) # ile wierzcholkow
    if(n<3):
        n=3
    
    wierzcholki=[]
    poz_x=0.0
    poz_y=0.0

    poz_x0=0.0
    poz_y0=0.0

    w=1

    #niezmienialne obrot
    kat_theta=0.0

    #flagi do rysowania i obracania
    pobranie_pozycji_szesciokata=True
    wierzcholek=False
    rysowanie_boku=False
    obracanie=False
    
    #parametry do rysowania bokow
    dystans=2.0

    #parametry do obracania
    szb_obr=45.0 #(stopnie/s)
    kat=((n-2)*180)/n   #stopnie
    obrot=180-kat
    faktyczny_obrot=obrot-1.3

    #niezmienialne konwersje
    katowa_prd = szb_obr*2*PI/360
    rel_kat_w = obrot*2*PI/360 #konwersja do pozycji wirzcholkow
    rel_kat = faktyczny_obrot*2*PI/360 # konwersja do obrotu
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

        #pobranie wspolrzednych kwadratu

        if(self.pobranie_pozycji_szesciokata):
            print('pobranie pozycji')
            self.poz_x0=self.poz_x
            self.poz_y0=self.poz_y
            self.wierzcholki.append([self.poz_x0,self.poz_y0])

            for i in range(self.n-1):
                self.wierzcholki.append([
                    self.wierzcholki[i][0]+math.cos(pose.theta+i*self.rel_kat_w)*self.dystans,
                    self.wierzcholki[i][1]+math.sin(pose.theta+i*self.rel_kat_w)*self.dystans])
                
            for i in range(len(self.wierzcholki)):
                print(self.wierzcholki[i])

            self.pobranie_pozycji_szesciokata=False
            self.wierzcholek=True

        #wierzchołek
        if(self.wierzcholek):
            
            self.x_goal=self.wierzcholki[self.w][0]
            self.y_goal=self.wierzcholki[self.w][1]
    
            self.w=self.w +1

            if(self.w > self.n-1):
                self.w = 0
            
            self.rysowanie_boku=True
            self.wierzcholek=False
            

        #rysowanie boku
            
        if(self.rysowanie_boku):
            K_linear = 2.0
            dis = abs(math.sqrt((((self.x_goal - pose.x) ** 2) + ((self.y_goal - pose.y) ** 2))))

            linear_speed = dis * K_linear

            K_angular = 4.0
            desired_angle_goal = math.atan2( self.y_goal - pose.y, self.x_goal-pose.x)
            angular_speed = (desired_angle_goal - pose.theta) * K_angular
            
            if(angular_speed>1.0):
                angular_speed=1.0
            if(angular_speed<-1.0):
                angular_speed=-1.0

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
    node_polygon = DrawPolygonNode()
    node_polygon.dystans=float(input('dlugosc boku?\n'))
    rclpy.spin(node_polygon)
    rclpy.shutdown()