#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class DrawCircleNode(Node):

    px=0.0
    py=0.0
    th=0.0

    def __init__(self):
        super().__init__("draw_circle_p")
        self.cmv_vel_pub_=self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.poz_sub=self.create_subscription(Pose,"/turtle1/pose",self.pose_callback,10)
        #self.timer=self.create_timer(0.1,self.move(4.0))
        self.move(2.0)
        self.obrot(90)
        self.get_logger().info("Draw circle node has been started p")

    def pose_callback(self,pose:Pose):
        self.px=pose.x
        self.py=pose.y
        self.th=pose.theta
        #print('a')

    def move(self,distance):
        print('jebac')
        msg=Twist()
        
        x_temp=self.px
        y_temp=self.py

        K_linear = 1.5
        K_angular = 4.0

        dis = abs(math.sqrt((((x_temp+distance-self.px) ** 2) + ((y_temp - self.py) ** 2))))
        linear_speed = dis * K_linear

            
        desired_angle_goal = math.atan2(y_temp - self.py, x_temp+distance-self.px)
        angular_speed = (desired_angle_goal - self.th) * K_angular
            
        if(angular_speed>1.0):
            angular_speed=1.0
        if(angular_speed<-1.0):
            angular_speed=-1.0

        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        self.cmv_vel_pub_.publish(msg)
        if (dis <= 0.01):
            print('koniec')
            msg.angular.z = 0.0
            self.obracanie=True
            self.rysowanie_boku=False

    def obrot(self,kat):
        msg=Twist()

        th_temp=self.th+kat*2*math.pi/360

        while not(th_temp<self.th):
            print(self.th)
            msg.angular.z=1.0
            self.cmv_vel_pub_.publish(msg)
        
        msg.angular.z=0.0
        self.cmv_vel_pub_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node_kolo = DrawCircleNode()
    rclpy.spin(node_kolo)
    #node_kolo.move(4)
    rclpy.shutdown()