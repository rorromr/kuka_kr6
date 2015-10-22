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
  #Light OFF 1
  kuka.ptp([0.06919094276104376, 0.8328329175439109, -0.9541489590049679, 1.2174451909374175e-05, 2.0049190529514756e-05, 4.534325117083477e-05]) 
  rospy.sleep(0.5)
  #Light OFF wait
  kuka.ptp([0.057203900823452614, 0.7638405342465404, -0.8145244175607402, -0.07516506634651865, -0.042651152866797215, 0.07450536806010266]) 
  rospy.sleep(0.5)
  #Light OFF 2
  kuka.ptp([0.06698597855359312, 0.805618141154234, -0.8794478236391257, 1.82616773561036e-05, 3.0073785794272135e-05, 4.551649317936866e-05]) 
  rospy.sleep(0.5)
  kuka.home()