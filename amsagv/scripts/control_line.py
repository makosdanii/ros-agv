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
distance_covered = 0

# Handle line sensor
def handleLine(msg):
  '''
  Handler for line sensor that controls speed and rotation,
  responsible for stopping the AGV if arrived
  
  :param msg: descritor object for the color sensor array's state
  '''
  global counter
  global direction
  global distance_covered
  msgCmdVel = Twist()

  k = 1 # the speed gain is decreased as target approached to avoid running off the line
  left = msg.line.left
  right = msg.line.right
  distance_covered += msg.line.distance # travelled distance is only integrated here/reset externally
  # print(f"{distance_covered=}", end='\r')
  # print(msg.line.left, msg.line.right, end='\r')

  # function returns early if no route has been given or it was completed
  # still responsible for keeping agv on the line
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
  
  # decide if distance_covered implies that virtual tag is reached,
  # slow down if final target is approached
  action = tags[counter].action
  if distance_covered >= action.distance - 0.03:
    if counter == len(tags) - 1:
      k = 0.35
    if action.id >= 100:
      print("Virtual tag hit")
      handleTag(None, virtual=True)
      return

  v = k*0.1

  # rotation is chosen depending on which side of the line 
  # we want to compensate the error to. this results in the robot following 
  # left or right paths
  direction = action.name
  if(direction == "left"):
    w = -1*(0.4-left)*5
  elif(direction == "right"):
    w = (0.4+right)*5
  elif direction == "pass":
    w = 0
  else:
    print("Invalid direction")

  # stop if the line is not detected anymore, meaning agv has run off the track
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
  '''
  this handler acts as an iterator for the tags list by incrementing counter
  :param msg: tag descriptor object
  :param virtual: defaults to false, 
    set explicitly to True when called for handling virtual tag
  '''
  global tag
  global counter
  global distance_covered

  if not virtual:
    tag = MTAG.get(msg.tag.id, None)
    print('New tag: {} -> {}'.format(msg.tag.id, tag))

    # check if we detected the expected tag of the route
    if tag == tags[counter].action.id:
      distance_covered = 0
      counter += 1
  
  else:
    # the function was called manually therefore no check just reset distance and increment
    distance_covered = 0
    counter += 1

  # check for overindexing
  if counter >= len(tags): 
    return
  
  action = tags[counter].action
  print(f"HandleTag: {action=}\n{counter=}")


def handleActions(msg):
  '''
  this handler will initialize variables/list for starting
  a new route
  '''
  global tags
  global counter
  counter = 0
  tags = msg.actions
  print('New path received')

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
