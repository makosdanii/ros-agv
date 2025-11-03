#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
from geometry_msgs.msg import Twist
from amsagv_msgs.msg import LineStamped, TagStamped
from math import pi, sin, cos, isnan
from world import MTAG
import math 



tag = None
direction  = 1 # 1=left -1=right

# Handle line sensor
def handleLine(msg):

  global direction
  
  print(msg.line.left, msg.line.right, end='\r')
  
  left = msg.line.left
  right = msg.line.right
  
  if(direction == 1):
    w = -1*(0.4-left)*5
    v = 0.1

  if(direction == -1):
    w = (0.4-(-1)*right)*5
    v = 0.1

  if math.isnan(left) or math.isnan(right):
    v=0
    w=0

  #tags assignment
  if(tag == 1):
    direction = 1
  elif(tag == 2):
    direction = -1

  # Velocity commands message
  msgCmdVel = Twist()
  msgCmdVel.linear.x = v
  msgCmdVel.angular.z = w
  # Publish velocity commands
  pubCmdVel.publish(msgCmdVel)



def handleTag(msg):
  global tag
  tag = MTAG.get(msg.tag.id, None)
  print('New tag: {} -> {}'.format(msg.tag.id, tag))



try:
  rospy.init_node('control_line')
  
  # Velocity commands publisher.
  pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  # Line sensor subscriber
  subLine = rospy.Subscriber('line', LineStamped, handleLine)
  # Tag subscriber
  subTag = rospy.Subscriber('tag', TagStamped, handleTag)

  rospy.spin()
except KeyboardInterrupt:
  pass
