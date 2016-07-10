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
        self.init_pose.position = Point(0.0, 0.0, 0.0)
        self.init_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.description = 'marker'
        self.scale = 0.3
        self.callback = OptionMarker.marker_feedback
        self.base_marker = None
    
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

    @staticmethod
    def void_callback(feedback):
        pass

class MarkerServer(object):
    """MarkerServer"""
    def __init__(self, topic = 'marker'):
        # Marker server
        self.server = InteractiveMarkerServer(topic)
        # Menu handler
        self.menu_handler = MenuHandler()
        
    def add_menu_entry(self, opt_menu=OptionMenu()):
        self.menu_handler.insert(opt_menu.entry_name, callback=opt_menu.callback)
        self.menu_handler.apply(self.server, opt_menu.marker_name)
        self.server.applyChanges()

    
    def add_6DOF(self, opt=OptionMarker()):
        marker = InteractiveMarker()
        marker.header.frame_id = opt.frame_id
        marker.pose = opt.init_pose
        marker.scale = opt.scale

        marker.name = opt.name
        marker.description = opt.description

        # Base control
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        if opt.base_marker:
            control.markers.append(opt.base_marker)
        marker.controls.append(control)

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

class MarkerColorBase(object):
    def __init__(self, r=1.0, g=1.0, b=1.0):
        self.r = r
        self.g = g
        self.b = b

class MarkerColor(object):
    WHITE = MarkerColorBase(1.0, 1.0, 1.0)
    GRAY  = MarkerColorBase(0.5, 0.5, 0.5)
    RED   = MarkerColorBase(0.9, 0.0, 0.0)
    GREEN = MarkerColorBase(0.0, 0.9, 0.0)
    BLUE  = MarkerColorBase(0.0, 0.0, 0.9)

def get_marker_sphere(scale=0.1, color=MarkerColor.WHITE, alpha=0.8):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = color.r
    marker.color.g = color.g
    marker.color.b = color.b
    marker.color.a = alpha
    return marker

def main():
    rospy.init_node('marker_test')
    rospy.loginfo('Init marker_test')
    marker_server = MarkerServer(topic = 'marker')

    marker_opt = OptionMarker()
    marker_opt.name = 'test_marker'
    marker_opt.base_marker = get_marker_sphere(color=MarkerColor.BLUE)

    menu_entry = OptionMenu()
    menu_entry.entry_name = 'Test marker'
    menu_entry.marker_name = 'test_marker'

    rospy.loginfo('Add 6DOF with menu')
    marker_server.add_6DOF(marker_opt)

    marker_server.add_menu_entry(menu_entry)

    rospy.spin()

if __name__ == "__main__":
    main()
