#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

"""
Interactive markers for control KUKA KR6
"""

__author__ = 'Rodrigo Muñoz'
__email__ = 'rmunozriffo@ing.uchile.cl'

import threading
from copy import copy

import rospy
from sensor_msgs.msg import JointState
from kuka_interface.commander import CommanderBase, Commander
from kuka_interface.marker import MarkerServer, OptionMarker, OptionMenu, get_marker_sphere
from visualization_msgs.msg import InteractiveMarkerFeedback

class KukaMarkerBaseController(object):
    """KukaMarkerBaseController"""
    def __init__(self):
        self.marker_server = MarkerServer(topic = 'kuka_marker')
        self.shadow_cmd = CommanderBase()
        self.joint_names = copy(self.shadow_cmd.joint_names)
        self.dof = len(self.joint_names)

        # Joint states for shadow robot
        self._joint_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        self._joint_states = copy(self.shadow_cmd.joint_states)

        self._add_ik_marker()
        self._add_menu()
        threading.Thread(target=self._publish_joint_states).start()


    def _publish_joint_states(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
          self._joint_states.header.stamp = rospy.Time.now()
          self._joint_pub.publish(self._joint_states)
          r.sleep()

    def _add_ik_marker(self):
        opt = OptionMarker()
        opt.name = 'ik_marker'
        opt.frame_id = 'shadow/base_link'
        opt.description = 'IK Marker for KUKA Robot'
        opt.init_pose = self.shadow_cmd.kinematics.forward_position_kinematics(self._joint_states.position)
        opt.callback = self.ik_marker_callback
        opt.base_marker = get_marker_sphere()
        
        self.marker_server.add_6DOF(opt)

    def _add_menu(self):
        # Get menu marker
        opt = OptionMenu()
        opt.marker_name = 'ik_marker'
        opt.entry_name = 'PTP'
        opt.callback = self.ptp

        self.marker_server.add_menu_entry(opt)
        

    def ptp(self, feedback):
        print(self._joint_states.position)

    def ik_marker_callback(self, feedback):
        # Check event
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # Update joint value using inverse kinematics
            result = self.shadow_cmd.kinematics.inverse_kinematics(
                feedback.pose.position, feedback.pose.orientation, self._joint_states.position)
            if result is not None:
                self._joint_states.position = result

class KukaMarkerController(KukaMarkerBaseController):
    def __init__(self):
        super(KukaMarkerController, self).__init__()
        self.kuka_cmd = Commander()
        
        
    def ptp(self, feedback):
        print('PTP')
        self.kuka_cmd.ptp(self._joint_states.position)

def main():
    rospy.init_node('marker_test')
    rospy.loginfo('Init marker_test')
    kuka_marker = KukaMarkerBaseController()
    rospy.spin()

if __name__ == "__main__":
    main()
