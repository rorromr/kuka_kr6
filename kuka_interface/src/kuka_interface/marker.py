#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

"""
Simple interactive marker server for 6DOF interactive markers
"""

__author__ = 'Rodrigo Muñoz'
__email__ = 'rmunozriffo@ing.uchile.cl'

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (InteractiveMarkerControl, InteractiveMarker,
    InteractiveMarkerFeedback)
from geometry_msgs.msg import Point


class Option6DOF(object):
    """Option6DOF"""
    def __init__(self):
        self.name = 'marker'
        self.frame_id = 'world'
        self.init_position = Point( 0.0, 0.0, 0.0)
        self.description = 'marker'
        self.scale = 0.3
        self.callback = Option6DOF.marker_feedback
    
    @staticmethod
    def marker_feedback(feedback):
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


class MarkerServer(object):
    """MarkerServer"""
    def __init__(self, topic = 'marker'):
        # Marker server
        self.server = InteractiveMarkerServer(topic)


    def add_6DOF(self, opt = Option6DOF()):
        marker = InteractiveMarker()
        marker.header.frame_id = opt.frame_id
        marker.pose.position = opt.init_position
        marker.scale = opt.scale

        marker.name = opt.name
        marker.description = opt.description

        # X axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = 'rotate_x'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)
        # X axis traslation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
        # Y axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = 'rotate_y'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)
        # Y axis traslation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
        # Z axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)
        # Z axis traslation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
        # Add marker to server
        self.server.insert(marker, opt.callback)
        self.server.applyChanges()


def main():
    rospy.init_node('marker_test')
    rospy.loginfo('Init marker_test')
    marker_server = MarkerServer(topic = 'marker')
    rospy.loginfo('Add 6DOF')
    marker_server.add_6DOF()

    rospy.spin()

if __name__ == "__main__":
    main()
