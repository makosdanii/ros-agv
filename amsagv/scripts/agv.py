#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
from agvapi import Agv, findLineEdges
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv_msgs.msg import LineStamped
import math 


with Agv() as robot:
  # Handle velocity commands
  def handleCmdVel(msg):
    global robot

    #print(msg)
    robot.setVel(msg.linear.x, msg.angular.z)



  try:
    rospy.init_node('agv')
    ns = rospy.get_namespace().lstrip('/')
    # Name of the odometry frame
    paramOdomFrameId = rospy.get_param('~odom_frame_id', '{}odom'.format(ns))
    # Name of the AGV frame
    paramAgvFrameId = rospy.get_param('~agv_frame_id', '{}agv'.format(ns))

    # Odometry publisher
    pubOdom = rospy.Publisher('odom', Odometry, queue_size=1)
    # Line sensor publisher
    pubLine = rospy.Publisher('line', LineStamped, queue_size=1)
    # Velocity commands subscriber.
    subCmdVel = rospy.Subscriber('cmd_vel', Twist, handleCmdVel)

    # Line-sensor message
    msgLine = LineStamped()

    # Odometry message
    msgOdom = Odometry()
    msgOdom.header.frame_id = paramOdomFrameId
    msgOdom.child_frame_id = paramAgvFrameId

    # Odometry initial state
    x, y, phi, gamma = 0.0, 0.0, 0.0, 0.0 # Robot configuration
    fd = 0.0 # Travelled distance of the front cart
    rate = rospy.Rate(50)

    right_wheel_samples = 112314
    left_wheel_samples = 114659.66666666667
    robot.readSensors()
    encLeft, encRight, encHeading = robot.getEncoders()

    sample_last_left, sample_last_right = encLeft, encRight
    sample_time = 0.02
    print()

    rotation_bias=4215
    
    while not rospy.is_shutdown():
      t = rospy.Time.now()
      
      # Read sensors
      robot.readSensors()

      # Encoders
      encLeft, encRight, encHeading = robot.getEncoders()

      #TODO Implement odometry here ...
      # print(f'Encoders: left={encLeft}, right={encRight}, heading={encHeading}'.format(encLeft, encRight, encHeading))

      #Rotation resolution is 8192 samples
      resolution =  2**13
      alfa = -2*math.pi*(encHeading-rotation_bias)/resolution

      sample_now_left = encLeft
      sample_now_right = encRight

      #sampletime 20ms 
      velocity_left=(sample_last_left-sample_now_left)/((sample_time)*left_wheel_samples)
      velocity_right=-(sample_last_right-sample_now_right)/((sample_time)*right_wheel_samples)

      sample_last_left = sample_now_left
      sample_last_right = sample_now_right

      velocity_avg = (velocity_left+velocity_right)/2

      x += velocity_avg * math.cos(alfa)*math.cos(phi)*sample_time
      y += velocity_avg * math.cos(alfa)*math.sin(phi)*sample_time
      phi += velocity_avg/0.12 * math.sin(alfa)*sample_time

      print(f'x position {x:10.10f}, y position {y:10.10f}, rotation{phi:10.10f}', end='\r')

      # Odometry message
      msgOdom.header.stamp = t
      msgOdom.pose.pose = ams.poseToPoseMsg(x, y, phi)
      msgOdom.pose.pose.position.z = alfa
      # Publish odometry message
      pubOdom.publish(msgOdom)

      #
      # Line sensor
      #

      # Line-sensor valuesPermission denied

      lineValues = robot.getLineValues()
      # Left and right line edge
      edgeLeft, edgeRight = findLineEdges(lineValues)

      # Line-sensor message
      msgLine.header.stamp = t
      msgLine.line.values = lineValues
      msgLine.line.left = edgeLeft if edgeLeft is not None else float('nan')
      msgLine.line.right = edgeRight if edgeRight is not None else float('nan')
      msgLine.line.heading = gamma
      msgLine.line.distance = fd
      # Publish line-sensor message
      pubLine.publish(msgLine)

      rate.sleep()
  except KeyboardInterrupt:
    pass
