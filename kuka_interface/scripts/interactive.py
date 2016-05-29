#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

"""
Interactive markers for control KUKA KR6
"""

__author__ = 'Rodrigo Muñoz'
__email__ = 'rmunozriffo@ing.uchile.cl'

import rospy
from kuka_interface.commander import CommanderBase
from kuka_interface.marker import MarkerServer, Option6DOF
from visualization_msgs.msg import InteractiveMarkerFeedback

class KukaMarkerController(object):
    """KukaMarkerController"""
    def __init__(self):
        self.marker_server = MarkerServer(topic = 'kuka_marker')
        self.kuka_cmd = CommanderBase()
        self._add_ik_marker()
        
    def _add_ik_marker(self):
        opt = Option6DOF()
        opt.name = 'ik_marker'
        opt.frame_id = 'shadow/base_link'
        opt.description = 'IK Marker for KUKA Robot'
        opt.callback = self.ik_marker_callback
        self.marker_server.add_6DOF(opt)

    def ik_marker_callback(self, feedback):
        rospy.loginfo('Feedback from ' + feedback.marker_name)
        # Check event
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( 'Pose changed')
            # Print marker position
            print( (feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.position.z) )
            # Print marker orientation
            print( (feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w) )


def main():
    rospy.init_node('marker_test')
    rospy.loginfo('Init marker_test')
    kuka_marker = KukaMarkerController()
    rospy.spin()

if __name__ == "__main__":
    main()
