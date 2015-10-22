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
  kuka.set_vel(60)
  kuka.home()
  rospy.sleep(0.5)
  #Light ON 1
  kuka.ptp([0.056919096553020854, 0.8133463016826425, -0.8952060805821915, -0.07516506634651865, -0.042651152866797215, 0.07450536806010266]) 
  rospy.sleep(0.5)
  #Light ON wait
  kuka.ptp([0.057203900823452614, 0.7638405342465404, -0.8145244175607402, -0.07516506634651865, -0.042651152866797215, 0.07450536806010266]) 
  rospy.sleep(0.5)
  #Light ON 2
  kuka.ptp([0.057203900823452614, 0.7876677697558776, -0.8279871628199942, -0.07516506634651865, -0.042651152866797215, 0.07450536806010266]) 
  rospy.sleep(0.5)
  kuka.home()