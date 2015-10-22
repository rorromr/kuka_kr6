#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz , David Valenzuela Urrutia'
__email__ = 'rorro.mr@gmail.com , david.valenzuela.u@gmail.com'

# ROS
import rospy
from kuka_driver.kuka_commander import KukaCommander
from sensor_msgs.msg import JointState
from std_msgs.msg import String

goal_pose_final_glob = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def callbackShadow(msg):
  global goal_pose_final_glob
  pose_shadow = msg.position 
  #print pose_shadow
  goal_pose_final = []
  goal_pose_final.append(pose_shadow[0])
  goal_pose_final.append(pose_shadow[1])
  goal_pose_final.append(pose_shadow[2])
  goal_pose_final.append(0.0)
  goal_pose_final.append(0.0)
  goal_pose_final.append(0.0)
  goal_pose_final_glob = goal_pose_final
  #print goal_pose_final_glob

if __name__=='__main__':

  rospy.init_node('follow_shadow', anonymous=True)
  rospy.Subscriber("shadow_joint_states", JointState, callbackShadow) 

  rate = rospy.Rate(3) # 3hz
  kuka = KukaCommander() 
  kuka.set_vel(70)
  while not rospy.is_shutdown():
    #global goal_pose_final_glob
    print goal_pose_final_glob
    kuka.ptp(goal_pose_final_glob)  
    rospy.sleep(0.1)
    rate.sleep()
    #rospy.spin()


  
  # kuka = KukaCommander()  
  # rospy.sleep(0.5)
  # kuka.set_vel(50)
  # kuka.home()
  # rospy.sleep(0.5)
  # kuka.ptp([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]) 
  # rospy.sleep(0.5)
  # kuka.home()
  # rospy.sleep(0.5)
  # kuka.ptp([0.11, 0.72, -0.8, 0.0, 0.0, 0.0]) #OPEN DOOR
  # kuka.home()