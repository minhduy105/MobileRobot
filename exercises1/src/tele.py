#!/usr/bin/env python

# Every python controller needs these lines
import rospy
import math
# The velocity command message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MovingForward(): 

    def __init__(self,stopD):
        self.stopD = stopD
        self.stopSignal = True

    def moving (self):
        rospy.init_node('tele')
        rospy.Subscriber('scan',LaserScan, self.callback1)
        #note: I delete the remap in the teleop_keyboard note
        rospy.Subscriber('turtlebot_teleop_keyboard/cmd_vel',Twist, self.callback2)

    def callback1(self,data1):
       
        #0.354 = width of the robot,  0.00159 = angle increment
        #calculate the width of the bot 
        calSize = int(math.ceil((0.354/self.stopD)/0.00159/2))  
        if min(data1.ranges[len(data1.ranges)/2-calSize:len(data1.ranges)/2+calSize]) < self.stopD:
            self.stopSignal = True 
        else:    
            self.stopSignal = False
      
            
    def callback2(self,data):
        pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1) 
        
        command = Twist()
        if self.stopSignal:
            command.linear.x = 0.0 
            command.linear.y = 0.0
            command.linear.z = 0.0

        else:    
            command.linear.x = data.linear.x
            command.linear.y = data.linear.y
            command.linear.z = data.linear.z
        #this mean it cannot go forward but cannot turn    
        command.angular.x = data.angular.x 
        command.angular.y = data.angular.y
        command.angular.z = data.angular.z


        # Loop at 10Hz, publishing movement commands until we shut down.
        rate = rospy.Rate(10)
        pub.publish(command)
        rate.sleep()


if __name__ == '__main__':
    stopD = rospy.get_param('/stopDistance')
    x = MovingForward(stopD)
    x.moving()
    rospy.spin()
