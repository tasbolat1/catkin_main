#! /usr/bin/env python

from __future__ import division, print_function

import rospy

import os
import time
import datetime
import numpy as np


from std_msgs.msg import Int16, String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState, Errors as FrankaErrors
import tf.transformations as tft

from franka_control_wrappers.panda_commander import PandaCommander

import dougsm_helpers.tf_helpers as tfh
from dougsm_helpers.ros_control import ControlSwitcher


class PandaOpenLoopGraspController(object):
    """
    Perform open-loop grasps from a single viewpoint using the Panda robot.
    """

    def __init__(self):


        self.LINK_EE_OFFSET = 0.27 # default 0.27

        self.curr_velocity_publish_rate = 100.0  # Hz
        self.curr_velo_pub = rospy.Publisher(
            "/cartesian_velocity_node_controller/cartesian_velocity",
            Twist,
            queue_size=1,
        )
        self.max_velo = 0.10
        self.curr_velo = Twist()

        self.cs = ControlSwitcher(
            {
                "moveit": "position_joint_trajectory_controller",
                "velocity": "cartesian_velocity_node_controller",
            }
        )
        self.cs.switch_controller("moveit")                            
        self.pc = PandaCommander(group_name="panda_arm")

        self.robot_state = None
        self.ROBOT_ERROR_DETECTED = False
        self.BAD_UPDATE = False
        self.USER_BUTTON_PRESSED = False
        rospy.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState,
            self.__robot_state_callback,
            queue_size=1,
        )

        # sub and pub for filename
        self.filename_pub = rospy.Publisher("filename", String, queue_size = 100)
        self.filename = None
        self.recording_till_stop = False

    def __filename_callback(self, msg):
        self.filename = msg.data
        print(self.filename)


        if self.filename == 'stop' and self.recording_till_stop:
            print('STOP command, returning to home')
            self.pc.recover()
            self.recording_till_stop = False

            self.cs.switch_controller("velocity")
            self.stop()
            self.cs.switch_controller("moveit")
            self.pc.goto_saved_pose("start_new", velocity=0.1)


        elif 'zero' in self.filename or 'pos30' in self.filename \
               or 'neg30' in self.filename and not self.recording_till_stop:
            print('Starting poking motion', self.filename)
            self.recording_till_stop = True
            self.go()
        else:
            print('Bad command')


    def __recover_robot_from_error(self):
        rospy.logerr("Recovering")
        self.pc.recover()
        self.cs.switch_controller("moveit")
        self.pc.goto_saved_pose("start", velocity=0.1)
        rospy.logerr("Done")
        self.ROBOT_ERROR_DETECTED = False

    def __robot_state_callback(self, msg):
        self.robot_state = msg
        
        if self.robot_state.robot_mode == 5:
            rospy.logerr("USER button pressed")
            self.USER_BUTTON_PRESSED = True

        if any(self.robot_state.cartesian_collision):
            if not self.ROBOT_ERROR_DETECTED:
                rospy.logerr("Detected Cartesian Collision")
            self.ROBOT_ERROR_DETECTED = True
        for s in FrankaErrors.__slots__:
            if getattr(msg.current_errors, s):
                self.stop()
                if not self.ROBOT_ERROR_DETECTED:
                    rospy.logerr("Robot Error Detected")
                self.ROBOT_ERROR_DETECTED = True
    def y_movement(self, degree):
        v = Twist()
        curr_pose = self.pc.get_current_pose()

        if degree == 0:
            return
        elif degree == 30:
            Y_OFFSET = -0.11
            V_LINEAR_Y = -0.05
            desired_y = curr_pose.position.y + Y_OFFSET
            v.linear.y = V_LINEAR_Y
            self.cs.switch_controller("velocity")
            print('here')
            while(self.robot_state.O_T_EE[-3] > desired_y
                    and not self.ROBOT_ERROR_DETECTED
                    and not self.USER_BUTTON_PRESSED
                    and self.recording_till_stop):
                self.curr_velo_pub.publish(v)
                print(self.robot_state.O_T_EE[-3], desired_y)
                rospy.sleep(0.01)
            self.stop()
            print('done!')
        elif degree == -30:
            Y_OFFSET = 0.05
            V_LINEAR_Y = 0.05

            desired_y = curr_pose.position.y + Y_OFFSET
            v.linear.y = V_LINEAR_Y
            self.cs.switch_controller("velocity")
            while(self.robot_state.O_T_EE[-3] < desired_y
                    and not self.ROBOT_ERROR_DETECTED
                    and not self.USER_BUTTON_PRESSED
                    and self.recording_till_stop):
                self.curr_velo_pub.publish(v)
                print(self.robot_state.O_T_EE[-3], desired_y)
                rospy.sleep(0.01)
            self.stop()


    def z_movement(self, degree):
        self.cs.switch_controller("velocity")
        v = Twist()
        VELOCITY = 0.02

        if degree == 0:
            Z_OFFSET = -0.5
            v.linear.z = -VELOCITY
        if degree ==30:
            Z_OFFSET = -0.45
            v.linear.z = -VELOCITY/(2**0.5)
            v.linear.y = VELOCITY/(2**0.5)

        if degree == -30:
            Z_OFFSET = -0.45
            v.linear.z = -VELOCITY/(2**0.5)
            v.linear.y = -VELOCITY/(2**0.5)


        curr_pose = self.pc.get_current_pose()
        desired_z = 0.12#0.11  for neutouch
        print(self.robot_state.O_T_EE[-2], desired_z)

        while (
            self.robot_state.O_T_EE[-2] > desired_z
            and not any(self.robot_state.cartesian_contact)
            and not self.ROBOT_ERROR_DETECTED
            and not self.USER_BUTTON_PRESSED
            and self.recording_till_stop
        ):
            self.curr_velo_pub.publish(v)
            rospy.sleep(0.01)
            print('running')
        print('z',not self.robot_state.O_T_EE[-2] > desired_z)
        print('contact?', any(self.robot_state.cartesian_contact))
        print('roboterorr?', self.ROBOT_ERROR_DETECTED)

        self.stop()




    def stop(self):
        self.pc.stop()
        self.curr_velo = Twist()
        self.curr_velo_pub.publish(self.curr_velo)

    def go(self):

        self.cs.switch_controller("moveit")
        self.pc.goto_saved_pose("start_new", velocity=0.1)

        curr_pose = self.pc.get_current_pose()
        print('current pose is', curr_pose)
        if 'pos30' in self.filename:
            DEGREE = 30
        elif 'neg30' in self.filename:
            DEGREE = -30
        elif 'zero' in self.filename:
            DEGREE = 0
        else:
            print('type degree in filename')
            return
        Y_OFFSET = 0.13
        v = Twist()

        if DEGREE == 0:
            self.pc.goto_saved_pose("start_new", velocity=0.1)
            self.z_movement(DEGREE)
        elif DEGREE == 30:
            self.pc.goto_saved_pose("start_pos30deg", velocity=0.1)
            self.y_movement(DEGREE)
            self.z_movement(DEGREE)
        elif DEGREE == -30:
            self.pc.goto_saved_pose("start_neg30deg", velocity=0.1)
            self.y_movement(DEGREE)
            self.z_movement(DEGREE)


        # print('yo!!')
        # if self.ROBOT_ERROR_DETECTED:
        #     rospy.logerr("Something went wrong, aborting this run")
        #     if self.ROBOT_ERROR_DETECTED:
        #         self.__recover_robot_from_error()
        


        # if self.USER_BUTTON_PRESSED:
        #     return

        # rospy.sleep(1)
        # # only publish stop after z_movement is done.
        # self.filename_pub.publish('stop')
        # rospy.sleep(1)

        # self.cs.switch_controller("moveit")
        # # self.pc.gripper.set_gripper(0.1)
        # self.pc.goto_saved_pose("start_new", velocity=0.1)

    def go_to_home(self, robotiq = False):
        if robotiq:
            self.pc.goto_saved_pose("home_robotiq", velocity=0.1)
        else:
            self.pc.goto_saved_pose("home", velocity=0.1)
        

    def move(self):
        self.cs.switch_controller("moveit")
        
        curr_pose = self.pc.get_current_pose()
        print('current pose is', curr_pose)

        self.go_to_home(robotiq = False)
        curr_pose = self.pc.get_current_pose()
        print('current pose is', curr_pose)

        return



    def start(self):
        print('waiting for messages.')
        rospy.Subscriber("filename", String, self.__filename_callback, queue_size = 1)
        rospy.spin()



if __name__ == "__main__":
    rospy.init_node("panda_poking")

    
    pg = PandaOpenLoopGraspController()
    pg.move()
    # pg.go()
    pg.start()
