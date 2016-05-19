#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

__author__ = 'Rodrigo Muñoz , David Valenzuela Urrutia'
__email__ = 'rorro.mr@gmail.com , david.valenzuela.u@gmail.com'

# ROS
import rospy
from sensor_msgs.msg import JointState
# KUKA Driver
from kuka_driver.srv import (
  SetVelocity, SetVelocityRequest, SetVelocityResponse,
  Home, HomeRequest, HomeResponse,
  Stop, StopRequest, StopResponse,
  PTP, PTPRequest, PTPResponse)
# KUKA Kinematics
from kuka_interface import kinematics
 
class CommanderBase(object):
  """Base class for commander"""
  def __init__(self):
    rospy.loginfo('Init KUKA Commander')
    self.ns = 'kuka_driver'

    # KUKA kinematics
    self._kin = kinematics.Kinematics(urdf_param = 'robot_description')
    # Joint states
    self._joint_sub = rospy.Subscriber('joint_states', JointState, self._joint_callback)
    self.joint_states = JointState()

  def _joint_callback(self, msg):
    # Update current joint states
    self.joint_states = msg

  def home(self):
    rospy.loginfo('home command called')
  
  def stop(self):
    rospy.loginfo('stop command called')
  
  def set_vel(self):
    rospy.loginfo('set_vel command called')
  
  def ptp(self):
    rospy.loginfo('ptp command called')


class Commander(CommanderBase):
  """Python interface for KUKA Robot"""

  def __init__(self):
    super(Commander,self).__init__()
    # Init interface with real robot using kuka_driver

    # Services
    rospy.wait_for_service(self.ns + '/home')
    self.home_server = rospy.ServiceProxy(self.ns+'/home', Home);
    self.stop_server = rospy.ServiceProxy(self.ns+'/stop', Stop);
    self.set_vel_server = rospy.ServiceProxy(self.ns+'/set_vel', SetVelocity);
    self.ptp_server = rospy.ServiceProxy(self.ns+'/ptp', PTP);

    # Base msgs
    self.home_req = HomeRequest()
    self.stop_req = StopRequest()
    self.set_vel_req = SetVelocityRequest()
    self.ptp_req = PTPRequest()

  def home(self):
    try:
      resp = self.home_server(self.home_req)
    except rospy.ServiceException as exc:
      print('Service "home" did not process request: ' + str(exc))

  def stop(self):
    try:
      resp = self.stop_server(self.stop_req)
    except rospy.ServiceException as exc:
      print('Service "stop" did not process request: ' + str(exc))

  def set_vel(self,desired_vel):
    self.set_vel_req.desired_vel = desired_vel
    try:
      resp = self.set_vel_server(self.set_vel_req)
    except rospy.ServiceException as exc:
      print('Service "set_vel" did not process request: ' + str(exc))

  def ptp(self, position):
    self.ptp_req.position = position
    try:
      resp = self.ptp_server(self.ptp_req)
    except rospy.ServiceException as exc:
      print('Service "ptp" did not process request: ' + str(exc))

def main():
  rospy.init_node('kuka_commander_test')
  kuka = CommanderBase()
  while not rospy.is_shutdown():
    print(kuka.joint_states.position)
    rospy.sleep(0.2)

if __name__ == '__main__':
  main()
