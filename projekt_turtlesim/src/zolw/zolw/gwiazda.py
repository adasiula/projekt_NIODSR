#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import math

PI = 3.1415926535897

#wierzcholki gwiazdy to nic innego 
# jak 2 pieciokąty tyle, że jeden w środku do góry nogami

class DrawStarNode(Node):

    #niezmienialne pozycja
    n=10 # ile wierzcholkow
    wierzcholki=[]
    wierzcholki_p1=[]
    wierzcholki_p2=[]

    poz_x=0.0
    poz_y=0.0

    poz_x0=0.0
    poz_y0=0.0

    x_temp=0.0
    y_temp=0.0

    w=1

    #niezmienialne obrot
    kat_theta=0.0
    

    #flagi do rysowania i obracania
    pobranie_pozycji_gwiazdy=True
    wierzcholek=False
    rysowanie_boku=False
    obracanie=False
    obracanie1=True
    obracanie2=False
    
    #parametry do rysowania bokow
    dystans=4.0

    #parametry do obracania
    # kat_szczytow=36
    kat_szczytow=float(input('kat szczytow (maks<54)?\n'))
    odchylenie=(108-kat_szczytow)/2
    szb_obr=45.0 #(stopnie/s)
    obrot1=180-(180-(108-kat_szczytow))
    faktyczny_obrot1=obrot1-1.3
    obrot2=180-kat_szczytow
    faktyczny_obrot1=obrot2-1.3

    #niezmienialne konwersje
    katowa_prd = szb_obr*2*PI/360
    odchylenie_w=odchylenie*2*PI/360
    rel_kat_w = (180-108)*2*PI/360 #konwersja do pozycji wirzcholkow pieciokata
    rel_kat1 = obrot1*2*PI/360 # konwersja do obrotu 1 
    rel_kat2 = obrot2*2*PI/360 # konwersja do obrotu 2
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

        if(self.pobranie_pozycji_gwiazdy):
            print('pobranie pozycji')
            self.poz_x0=self.poz_x
            self.poz_y0=self.poz_y
            self.wierzcholki_p1.append([self.poz_x0,self.poz_y0])

            #pieciokat1 - duzy
            for i in range(4):
                self.wierzcholki_p1.append([
                    self.wierzcholki_p1[i][0]+math.cos(pose.theta-i*self.rel_kat_w+self.odchylenie_w)*self.dystans,
                    self.wierzcholki_p1[i][1]+math.sin(pose.theta-i*self.rel_kat_w+self.odchylenie_w)*self.dystans])


            #pieciokat2
            przesuniecie=(self.dystans/2)/math.sin(((180-(108-self.kat_szczytow))/2)*(2*PI/360))
            maly_bok=(2*math.sin(108*PI/360)*self.dystans-2*przesuniecie)
            print(self.dystans)
            print(przesuniecie)
            print(maly_bok)


            self.x_temp=self.poz_x0+math.cos(pose.theta)*przesuniecie
            self.y_temp=self.poz_y0+math.sin(pose.theta)*przesuniecie
            self.wierzcholki_p2.append([self.x_temp,self.y_temp])

            for i in range(4):
                self.wierzcholki_p2.append([
                    self.wierzcholki_p2[i][0]+math.cos(pose.theta-i*self.rel_kat_w)*(maly_bok),
                    self.wierzcholki_p2[i][1]+math.sin(pose.theta-i*self.rel_kat_w)*(maly_bok)])

            p1=0
            p2=0

            for i in range(len(self.wierzcholki_p2)+len(self.wierzcholki_p1)):
                print(len(self.wierzcholki_p1))
                if(i%2==0):
                    self.wierzcholki.append(self.wierzcholki_p1[p1])
                    p1=p1+1
                if(i%2==1):
                    self.wierzcholki.append(self.wierzcholki_p2[p2])
                    p2=p2+1

            for i in range(len(self.wierzcholki)):
                print(self.wierzcholki[i])



            self.poz_x0=self.poz_x
            self.poz_y0=self.poz_y
            self.wierzcholki.append([self.poz_x0,self.poz_y0])

            for i in range(self.n-1):
                self.wierzcholki.append([
                    self.wierzcholki[i][0]+math.cos(pose.theta+i*self.rel_kat_w)*self.dystans,
                    self.wierzcholki[i][1]+math.sin(pose.theta+i*self.rel_kat_w)*self.dystans])
                
            for i in range(len(self.wierzcholki)):
                print(self.wierzcholki[i])

            self.pobranie_pozycji_gwiazdy=False
            self.wierzcholek=True

        #wierzchołek
        if(self.wierzcholek):
            
            self.x_goal=self.wierzcholki[self.w][0]
            self.y_goal=self.wierzcholki[self.w][1]
    
            self.w=self.w +1

            if(self.w > 9):
                self.w = 0
            
            self.rysowanie_boku=True
            self.wierzcholek=False
            

        #rysowanie boku
            
        if(self.rysowanie_boku):
            K_linear = 1.5
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

            obecny_kat=0.0
            kat_rzadany=0.0

            if(self.obracanie1):
                velocity_message.angular.z= abs(self.katowa_prd)
                kat_rzadany=self.rel_kat1
                self.obracanie2=True
                self.obracanie1=False
            elif(self.obracanie2):
                velocity_message.angular.z= -abs(self.katowa_prd)
                kat_rzadany=self.rel_kat2
                self.obracanie1=True
                self.obracanie2=False
            # self.obracanie1=not self.obracanie1
            # self.obracanie2=not self.obracanie2

            
            self.t0=time.time()
            while(obecny_kat<kat_rzadany):
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
    node_star = DrawStarNode()
    node_star.dystans=float(input('odleglosc miedzy wierzcholkami?\n'))
    rclpy.spin(node_star)
    rclpy.shutdown()