#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan 
from geometry_msgs.msg import Twist
import random

disToObstacle = 0.5

def callback(msg): 
  rospy.loginfo(rospy.get_caller_id() + " The distance to obstacle is -  %s",msg.ranges[300]) #prints on terminal

#If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
  if msg.ranges[300] > disToObstacle:
      move.linear.x = random.uniform(0, 0.2)
      move.linear.y = random.uniform(0, 0.1)
      move.angular.z = 0

#If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will turn left
  if msg.ranges[300] <= disToObstacle: 
      move.linear.x = 0
      move.linear.y = 0
      move.angular.z = 0.5

  pub.publish(move)
  

rospy.init_node('sub_node')
sub = rospy.Subscriber('/scan', LaserScan, callback) #We subscribe to the laser's topic
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
rate = rospy.Rate(2)
move = Twist()


rospy.spin()