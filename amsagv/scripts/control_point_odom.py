#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math




# Handle odometry
def handleOdometry(msg):

  D = 0.1207 # 120.7mm
  x, y, phi = ams.msgToPose(msg.pose.pose)
  gamma = msg.pose.pose.position.z
  #print(x, y, phi, gamma)

  x_ref, y_ref = 1, 1 #desired point  

  # CTP controller 
  # INPUT - x_ref, y_ref, x, y, phi
  # OUTPUT v, w

  K_w=4
  K_v=1
  K_ws=4
  phi_ref = math.atan2((y_ref-y) , (x_ref-x))
  w = K_w*ams.wrapToPi(phi_ref-phi)
  distance_error = math.sqrt( math.pow(x_ref-x, 2) + math.pow(y_ref-y, 2) )
  if distance_error < 0.02:
    K_v = 0
    K_ws = 0

  v = K_v*distance_error


  # INPUT v, w
  # OUTPUT gamma_ref, vs
  gamma_ref = math.atan2(w*D, v)
  vs = math.sqrt(math.pow(v, 2) + math.pow(w, 2)*math.pow(D, 2))

  # INPUT gamma, gamma_ref
  # OUTPUT ws
  gamma_err = ams.wrapToPi(gamma_ref-gamma)
  ws = K_ws*gamma_err

  # keeping velocity within limits
  if vs>0.2:
     vs=0.2
    
  print(f'vs {vs:10.10f}, ws {ws:10.10f}, phi_ref{phi_ref:10.10f}' , end='\r')

  # Velocity commands message
  msgCmdVel = Twist()
  msgCmdVel.linear.x = vs
  msgCmdVel.angular.z = ws
  # Publish velocity commands
  pubCmdVel.publish(msgCmdVel)

print()
try:
  rospy.init_node('control_line')
  
  # Velocity commands publisher.
  pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  # Odometry subscriber
  subOdom = rospy.Subscriber('odom', Odometry, handleOdometry)

  rospy.spin()
except KeyboardInterrupt:
    pass
