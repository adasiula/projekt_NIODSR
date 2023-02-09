
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
    poz_x1=0.0
    poz_y1=0.0
    distacne_moved = 0.0

    #niezmienialne obrot
    obrot=0.0
    obr=0.0
    error=0.05

    #flagi do rysowania i obracania
    zdobycie_pozycji=True
    jazda1=False
    jazda2=False
    jazda3=False
    jazda4=False

    #parametry do rysowania bokow
    speed=1.0
    distance=4.0
    is_forward=True

    #parametry do obracania
    szb_obr=15.0 #(degrees/s)
    kat=88.7 #degrees

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
        print('x y theta')
        print(self.poz_x)
        print(self.poz_y)
        print(self.obr)
        

        velocity_message = Twist()

        
        if(self.zdobycie_pozycji):
            self.poz_x0=pose.x
            self.poz_y0=pose.y
            self.poz_x1=pose.x+self.distance
            self.poz_y1=pose.y#+self.distance
            self.zdobycie_pozycji=False
            self.jazda1=True
        
        if(self.jazda1):
            K_linear = 0.5
            dis = abs(math.sqrt((((self.poz_x1 - pose.x) ** 2) + ((self.poz_y1 - pose.y) ** 2))))

            linear_speed = dis * K_linear

            K_angular = 4.0
            desired_angle_goal = math.atan2( self.poz_y1 - pose.y, self.poz_x1-pose.x)
            angular_speed = (desired_angle_goal - pose.theta) * K_angular

            velocity_message.linear.x = linear_speed
            velocity_message.angular.z = angular_speed

            self.cmv_vel_pub_.publish(velocity_message)
            if (dis <= 0.01):
                self.jazda1=False
                self.jazda2=True
        




            



            


            

            


def main(args=None):
    rclpy.init(args=args)
    node_square = DrawSquareNode()
    # node_square.speed=2.0
    # node_square.distance=2.0
    # node_square.is_forward=True
    #node_square.send_velocity_command(1.0,1.0,True)
    rclpy.spin(node_square)
    #node_square.send_velocity_command(1.0,4.0,True)
    
    #node_square.is_forward=False
    #rclpy.spin(node_square)
    rclpy.shutdown()