#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

"""
PyKDL based KUKA kinematics
"""

__author__ = 'Rodrigo Muñoz'
__email__ = 'rmunozriffo@ing.uchile.cl'

import rospy
import numpy as np

from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL

class Kinematics(object):
    def __init__(self, urdf_param = 'robot_description'):
        self._urdf = URDF.from_parameter_server(urdf_param)
        (parse_ok, self._kdl_tree) = treeFromUrdfModel(self._urdf)
        # Check @TODO Exception
        if not parse_ok:
            rospy.logerr('Error parsing URDF from parameter server ({0})'.format(urdf_param))
        else:
            rospy.logdebug('Parsing URDF succesful')

        self._base_link = self._urdf.get_root()
        # @TODO Hardcoded
        self._tip_link = 'link_6'
        self._tip_frame = PyKDL.Frame()
        self._arm_chain = self._kdl_tree.getChain(self._base_link,
                                                  self._tip_link)
        # @TODO Hardcoded
        self._joint_names = ['a1', 'a2', 'a3', 'a4', 'a5', 'a6']
        self._num_joints = len(self._joint_names)

        # KDL Solvers
        self._fk_p_kdl = PyKDL.ChainFkSolverPos_recursive(self._arm_chain)
        self._fk_v_kdl = PyKDL.ChainFkSolverVel_recursive(self._arm_chain)
        self._ik_v_kdl = PyKDL.ChainIkSolverVel_pinv(self._arm_chain)
        self._ik_p_kdl = PyKDL.ChainIkSolverPos_NR(self._arm_chain,
                                                   self._fk_p_kdl,
                                                   self._ik_v_kdl)
        self._jac_kdl = PyKDL.ChainJntToJacSolver(self._arm_chain)
        self._dyn_kdl = PyKDL.ChainDynParam(self._arm_chain,
                                            PyKDL.Vector.Zero())

    def print_robot_description(self):
        nf_joints = 0
        for j in self._urdf.joints:
            if j.type != 'fixed':
                nf_joints += 1
        print('URDF non-fixed joints: %d' % nf_joints)
        print('URDF total joints: %d' % len(self._urdf.joints))
        print('URDF links: %d' % len(self._urdf.links))
        print('KDL joints: %d' % self._kdl_tree.getNrOfJoints())
        print('KDL segments: %d' % self._kdl_tree.getNrOfSegments())

    def print_kdl_chain(self):
        for idx in xrange(self._arm_chain.getNrOfSegments()):
            print(self._arm_chain.getSegment(idx).getName())


    def joints_to_kdl(self, type = 'positions', values = None):
        kdl_array = PyKDL.JntArray(self._num_joints)

        if values is None:
            values = np.zeros((self._num_joints, 1))
        
        for idx, name in enumerate(self._joint_names):
            kdl_array[idx] = values[idx]
        if type == 'velocities':
            kdl_array = PyKDL.JntArrayVel(kdl_array)
        return kdl_array
    
    def _kdl_to_mat(self, data):
        mat =  np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i,j] = data[i,j]
        return mat

    def forward_position_kinematics(self, joint_values=None):
        end_frame = PyKDL.Frame()
        self._fk_p_kdl.JntToCart(self.joints_to_kdl('positions',joint_values),
                                 end_frame)
        pos = end_frame.p
        rot = PyKDL.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        return np.array([pos[0], pos[1], pos[2],
                         rot[0], rot[1], rot[2], rot[3]])

    def forward_velocity_kinematics(self,joint_velocities=None):
        end_frame = PyKDL.FrameVel()
        self._fk_v_kdl.JntToCart(self.joints_to_kdl('velocities',joint_velocities),
                                 end_frame)
        return end_frame.GetTwist()
    
    def inverse_kinematics(self, position, orientation=None, seed=None):
        pos = PyKDL.Vector(position[0], position[1], position[2])
        if orientation != None:
            rot = PyKDL.Rotation()
            rot = rot.Quaternion(orientation[0], orientation[1],
                                 orientation[2], orientation[3])
        # Populate seed with current angles if not provided
        seed_array = PyKDL.JntArray(self._num_joints)
        if seed != None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')

        # Make IK Call
        if orientation != None:
            goal_pose = PyKDL.Frame(rot, pos)
        else:
            goal_pose = PyKDL.Frame(pos)
        result_angles = PyKDL.JntArray(self._num_joints)

        if self._ik_p_kdl.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles))
            return result
        else:
            return None

    def jacobian(self,joint_values=None):
        jacobian = PyKDL.Jacobian(self._num_joints)
        self._jac_kdl.JntToJac(self.joints_to_kdl('positions',joint_values), jacobian)
        return self._kdl_to_mat(jacobian)

    def jacobian_transpose(self,joint_values=None):
        return self.jacobian(joint_values).T

    def jacobian_pseudo_inverse(self,joint_values=None):
        return np.linalg.pinv(self.jacobian(joint_values))

    def inertia(self,joint_values=None):
        inertia = PyKDL.JntSpaceInertiaMatrix(self._num_joints)
        self._dyn_kdl.JntToMass(self.joints_to_kdl('positions',joint_values), inertia)
        return self._kdl_to_mat(inertia)

    def cart_inertia(self,joint_values=None):
        js_inertia = self.inertia(joint_values)
        jacobian = self.jacobian(joint_values)
        return np.linalg.inv(jacobian * np.linalg.inv(js_inertia) * jacobian.T)

if __name__ == '__main__':
    rospy.init_node('kuka_kinematics_test')
    kinematics = Kinematics()
    kinematics.print_robot_description()
    kinematics.print_kdl_chain()
    print('IK Test')
    p = kinematics.forward_position_kinematics(joint_values=[0.1,0,0,0,0,0.0])
    pos = p[0:3]
    rot = p[3:]
    print(pos)
    print(rot)
    print(kinematics.inverse_kinematics(pos,rot,[0,0,0,0,0,0.0]))
    print('Inertia')
    print(kinematics.jacobian_pseudo_inverse([0,0,0,0,0,0.0]))
