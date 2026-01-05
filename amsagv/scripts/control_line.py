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
arrived = False
distance_covered = 0

# Handle line sensor
def handleLine(msg):
  global arrived
  global counter
  global direction
  global distance_covered
  msgCmdVel = Twist()

  k = 1
  left = msg.line.left
  right = msg.line.right
  distance_covered += msg.line.distance
  #print(f"{distance_covered=}", end='\r')
  # print(msg.line.left, msg.line.right, end='\r')

  if tags is None or counter >= len(tags):
    print("Arrived", end='\r')
    v = 0
    w = -1*(0.4-left)*5
    # Velocity commands message
    msgCmdVel.linear.x = v
    msgCmdVel.angular.z = w
    # Publish velocity commands
    pubCmdVel.publish(msgCmdVel)
    return
  
  action = tags[counter].action
  if distance_covered >= action.distance - 0.03:
    if counter == len(tags) - 1:
      k = 0.35
    #print(f"\n{action.id=}")
    if action.id >= 100:
      print("Virtual tag hit")
      handleTag(None, virtual=True)
      return

  direction = action.name
  
  v = k*0.1
  if(direction == "left"):
    w = -1*(0.4-left)*5
  elif(direction == "right"):
    w = (0.4-(-1)*right)*5
  elif direction == "pass":
    w = 0
  else:
    print("I'm in danger")

  if math.isnan(left) or math.isnan(right):
    print("Lost line", end='\r')
    v=0
    w=0

  # Velocity commands message
  msgCmdVel.linear.x = v
  msgCmdVel.angular.z = w
  # Publish velocity commands
  pubCmdVel.publish(msgCmdVel)



def handleTag(msg, virtual = False):
  global tag
  global counter
  global arrived
  global distance_covered

  if not virtual:
    tag = MTAG.get(msg.tag.id, None)
    print('New tag: {} -> {}'.format(msg.tag.id, tag))

    if tag == tags[counter].action.id:
      distance_covered = 0
      counter += 1
  
  else:
    distance_covered = 0
    counter += 1

  if counter >= len(tags): 
    arrived = True
    return
  
  action = tags[counter].action
  print(f"HandleTag: {action=}\n{counter=}")

  


def handleActions(msg):
  global tags
  global arrived
  global counter
  arrived = False
  print('New path received')
  counter = 0
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
