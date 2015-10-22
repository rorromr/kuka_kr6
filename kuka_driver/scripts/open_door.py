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
  rospy.sleep(0.5)
  kuka.ptp([0.10772384992907114, 0.7154650789473394, -0.8016527609008506, -6.087224938771763e-06, 1.0266153515233172e-05, 6.821469104602556e-05]) #OPEN DOOR
  rospy.sleep(0.5)
  kuka.ptp([0.10770660596114645, 0.7292428761611092, -0.801648366685086, -6.087224938771763e-06, 2.0411520793821173e-05, 6.866109236713328e-05]) #OPEN DOOR
  rospy.sleep(0.5)
  kuka.home()