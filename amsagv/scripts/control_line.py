#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
from geometry_msgs.msg import Twist
from amsagv_msgs.msg import LineStamped, TagStamped, ActionsStamped
from math import pi, sin, cos, isnan
from world import MTAG
import math 



counter = 0
tag = None
tags = None

# Handle line sensor
def handleLine(msg):

  global direction

  print(msg.line.left, msg.line.right, end='\r')
  
  left = msg.line.left
  right = msg.line.right

  # comes from msg.actions.action.name
  action = tags[counter].action
  direction = action.name
  
  if(direction == "left"):
    w = -1*(0.4-left)*5
    v = 0.1

  if(direction == "right"):
    w = (0.4-(-1)*right)*5
    v = 0.1

  if math.isnan(left) or math.isnan(right):
    v=0
    w=0

  if tag == tags[-1].action.id:
    v=0
    w=0

  # Velocity commands message
  msgCmdVel = Twist()
  msgCmdVel.linear.x = v
  msgCmdVel.angular.z = w
  # Publish velocity commands
  pubCmdVel.publish(msgCmdVel)



def handleTag(msg):
  global tag
  global counter
  counter += 1
  action = tags[counter].action
  print(f"{action=}")
  tag = MTAG.get(msg.tag.id, None)
  print('New tag: {} -> {}'.format(msg.tag.id, tag))

def handleActions(msg):
  global tags
  tags = msg.actions


try:
  rospy.init_node('control_line')
  
  # Velocity commands publisher.
  pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  # Line sensor subscriber
  subLine = rospy.Subscriber('line', LineStamped, handleLine)
  # Tag subscriber
  subTag = rospy.Subscriber('tag', TagStamped, handleTag)
  # Actions subscriber
  subActions = rospy.Subscriber('path_actions', ActionsStamped, handleActions)

  rospy.spin()
except KeyboardInterrupt:
  pass
