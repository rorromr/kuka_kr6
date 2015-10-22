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
  kuka.set_vel(80)
  kuka.home()