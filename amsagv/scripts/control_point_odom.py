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

  x_ref, y_ref = -0.4, 0.4 #desired point  

  # control to ref.pose using intermediate direction
  final_angle = 0
  radius_at_approach = 0.3

  # cosine initial error clamping
  threshold_angle_error = math.pi/2
  def clamp_orientation_error(error):
    R = math.pow(math.cos(error), 3)
    if abs(error) > threshold_angle_error:
      R = 0
    return R

  # CTP controller 
  # INPUT - x_ref, y_ref, x, y, phi
  # OUTPUT v, w

  K_w=4
  K_v=1
  K_ws=4
  phi_ref = math.atan2((y_ref-y) , (x_ref-x))
  
  distance_error = math.sqrt( math.pow(x_ref-x, 2) + math.pow(y_ref-y, 2) )
  
  alpha = ams.wrapToPi(phi_ref - final_angle)
  beta = math.atan2(radius_at_approach, distance_error) * (-1 if alpha < 0 else 1)
  if abs(alpha) > abs(beta):
    final_angle_error = ams.wrapToPi(phi_ref - phi + beta)
  else: 
    final_angle_error = ams.wrapToPi(phi_ref - phi + alpha)
  

  w = K_w*final_angle_error

  if distance_error < 0.02:
    K_v = 0
    K_ws = 0

  R = clamp_orientation_error(final_angle_error)
  v = R*K_v*distance_error

  # INPUT v, w
  # OUTPUT gamma_ref, vs
  gamma_ref = math.atan2(w*D, v)

  # INPUT gamma, gamma_ref
  # OUTPUT ws
  gamma_err = ams.wrapToPi(gamma_ref-gamma)
  ws = K_ws*gamma_err

  R = clamp_orientation_error(gamma_err)
  vs = math.sqrt(math.pow(v, 2) + math.pow(w, 2)*math.pow(D, 2))*R

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
