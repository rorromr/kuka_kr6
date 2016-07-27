#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz , David Valenzuela Urrutia'
__email__ = 'rorro.mr@gmail.com , david.valenzuela.u@gmail.com'

# ROS
import rospy
from kuka_driver.kuka_commander import KukaCommander

if __name__=='__main__':
  rospy.init_node('kuka_commander')

  kuka = KukaCommander()  
  rospy.sleep(0.5)
  kuka.set_vel(30)
  kuka.home()
  rospy.sleep(0.5)
  kuka.ptp([0.2, 0.0, 0.0, 0.0, 0.0, 0.0]) 
  rospy.sleep(0.5)
  kuka.home()
  rospy.sleep(0.5)
  kuka.ptp([-0.2, 0.0, 0.0, 0.0, 0.0, 0.0]) #OPEN DOOR
  kuka.home()
