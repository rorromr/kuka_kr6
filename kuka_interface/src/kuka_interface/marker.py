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
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (Marker, InteractiveMarkerControl, InteractiveMarker,
    InteractiveMarkerFeedback)
from geometry_msgs.msg import Point, Quaternion, Pose


class OptionMarker(object):
    """OptionMarker"""
    def __init__(self):
        self.name = 'marker'
        self.frame_id = 'world'
        self.init_pose = Pose()
        self.init_pose.position = Point( 0.0, 0.0, 0.0)
        self.init_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.description = 'marker'
        self.scale = 0.3
        self.callback = OptionMarker.marker_feedback
    
    @staticmethod
    def marker_feedback(feedback):
        rospy.loginfo('Feedback from ' + feedback.marker_name)
        # Check event
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo('Pose changed')
            # Print marker position
            print( (feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.position.z) )
            # Print marker orientation
            print( (feedback.pose.orientation.x,
                feedback.pose.orientation.y,
                feedback.pose.orientation.z,
                feedback.pose.orientation.w) )

    @staticmethod
    def void_callback(feedback):
        pass

class OptionMenu(object):
    """OptionMenu"""
    def __init__(self):
        self.marker_name = 'marker'
        self.entry_name = 'menu'
        self.callback = OptionMenu.menu_callback
    
    @staticmethod
    def menu_callback(feedback):
        rospy.loginfo('Feedback from ' + feedback.marker_name)
        # Check event
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            # Grasp
            rospy.loginfo('Menu selected: ' + str(feedback.menu_entry_id))


class MarkerServer(object):
    """MarkerServer"""
    def __init__(self, topic = 'marker'):
        # Marker server
        self.server = InteractiveMarkerServer(topic)
        # Menu handler
        self.menu_handler = MenuHandler()

    @staticmethod
    def get_menu_marker(opt_marker = OptionMarker()):
        # Menu marker
        marker = InteractiveMarker()
        marker.header.frame_id = opt_marker.frame_id
        marker.pose = opt_marker.init_pose
        marker.scale = opt_marker.scale
        marker.name = opt_marker.name
        # Void sphere marker
        void_marker = Marker()
        void_marker.type = Marker.SPHERE
        void_marker.scale.x = opt_marker.scale
        void_marker.scale.y = opt_marker.scale
        void_marker.scale.z = opt_marker.scale
        void_marker.color.r = 0.5
        void_marker.color.g = 0.5
        void_marker.color.b = 0.5
        void_marker.color.a = 0.5
        # Add control
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(void_marker)
        marker.controls.append(control)
        return marker

        
    def add_menu_entry(self, marker=None, opt_menu=OptionMenu()):
        if not marker:
            marker = self.get_menu_marker()
        self.server.insert(marker, OptionMarker.void_callback)
        self.server.applyChanges()
        self.menu_handler.insert(opt_menu.entry_name, callback=opt_menu.callback)
        self.menu_handler.apply(self.server, opt_menu.marker_name)
        self.server.applyChanges()

    
    def add_6DOF(self, opt = OptionMarker()):
        marker = InteractiveMarker()
        marker.header.frame_id = opt.frame_id
        marker.pose = opt.init_pose
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
