#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz , David Valenzuela Urrutia'
__email__ = 'rorro.mr@gmail.com , david.valenzuela.u@gmail.com'

# ROS
import rospy
from kuka_driver.srv import (
  SetVelocity, SetVelocityRequest, SetVelocityResponse,
  Home, HomeRequest, HomeResponse,
  Stop, StopRequest, StopResponse,
  PTP, PTPRequest, PTPResponse)
 
# Thread
from threading import Thread

class KukaCommander():
  """KukaCommander Class for Kuka Robot"""

  def __init__(self):
    rospy.loginfo('Init Kuka Commander')
    self.ns = "kuka_driver"
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
    # Publish msg on thread
    # self.state_update_rate = 10
    # self.is_running = True
    # self.pub_thread = Thread(target=self.update_state)
    # self.pub_thread.start()

  def home(self):
    try:
      resp = self.home_server(self.home_req)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

  def stop(self):
    try:
      resp = self.stop_server(self.stop_req)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

  def set_vel(self,desired_vel):
    self.set_vel_req.desired_vel = desired_vel
    try:
      resp = self.set_vel_server(self.set_vel_req)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))

  def ptp(self, position):
    self.ptp_req.position = position
    try:
      resp = self.ptp_server(self.ptp_req)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))