#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import yaml
import time
import os
import csv
import numpy as np
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg
from moveit_msgs.msg import RobotTrajectory
from controller_manager_msgs.srv import SwitchController
import pygame

import franka_gripper.msg as fg
import actionlib

class MoveGrasp(object):
    def __init__(self):
        super(MoveGrasp, self).__init__()
        rospy.init_node('move_grasp_python', anonymous=True)
        self.filename_pub = rospy.Publisher('/filename', String, queue_size = 1000) #publishing here saves robotiq+franka+aces+realsense+prophesee data.
        self.rbtq_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg, queue_size = 1000)
        self.cmd_sub = rospy.Subscriber('/target', String, self.cmdCallback, queue_size=1000)
        self.controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        # Misc variables
        pygame.init()
        pygame.mixer.init()
        self.sound_directories = ['/home/crslab/panda_sim/panda_examples/correct.ogg','/home/crslab/panda_sim/panda_examples/warning.ogg','/home/crslab/panda_sim/panda_examples/try_again.ogg']
        self.lock = False
        self.is_moveit_inited = False
        self.f = None
        self.home_quat = np.array([0.92416610616, -0.381986892882, -0.000972960439309, 0.00144056942286]) / np.linalg.norm(np.array([0.92416610616, -0.381986892882, -0.000972960439309, 0.00144056942286]))
        self.traj_timestamps_holder = list() #list which store routine start/end times.

    def move_it_init(self):
        if self.is_moveit_inited:
            return
        self.is_moveit_inited = True

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "panda_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def all_joint_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    print("ABOVE TOLERANCE")
                    for i in range(len(goal)):
                        print(i, actual[i] , goal[i])
                    return False
                else:
                    pass
                    #print(index, "   ",abs(actual[index] - goal[index]))

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_joint_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_joint_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

    def all_cartesian_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        all_equal = True
        if type(goal) is list:
            for index in range(7):
                if index <= 2 and abs(actual[index] - goal[index]) > tolerance: #for position
                    print("ABOVE TOLERANCE")
                    for i in range(len(goal)):
                        print(i, actual[i] , goal[i])
                    return False
                elif index > 2 and min(abs(actual[index] - goal[index]), abs(actual[index] + goal[index])) > tolerance: #for orientation. q and -q are the same.
                    print("ABOVE TOLERANCE")
                    for i in range(len(goal)):
                        print(i, actual[i] , goal[i])
                    return False
                else:
                    pass
                    #print(index, "   ",abs(actual[index] - goal[index]))

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_cartesian_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_cartesian_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

    def pose_check(self, desired_xyz, desired_quat):
        '''
        Checks current cartesian pose against desired cartesian pose.
        '''
        current_pose = self.group.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = desired_quat[0]
        pose_goal.orientation.y = desired_quat[1]
        pose_goal.orientation.z = desired_quat[2]
        pose_goal.orientation.w = desired_quat[3]
        pose_goal.position.x = desired_xyz[0]
        pose_goal.position.y = desired_xyz[1]
        pose_goal.position.z = desired_xyz[2]
        return self.all_cartesian_close(pose_goal, current_pose, 0.005)

    def joint_check(self, desired_joints):
        '''
        Checks current joint pose against desired joint pose.
        '''
        joint_goal = self.group.get_current_joint_values()
        for joint_idx in range(7):
            joint_goal[joint_idx] = desired_joints[joint_idx]
        current_joints = self.group.get_current_joint_values()
        return self.all_joint_close(joint_goal, current_joints, 0.005)


    def go_to_home(self, robotiq = True):
        '''
        Plans and execute a trajectory to home position. 
        Franka arm + Franka Gripper has a different home position from Franka arm + Robotiq 2F-140.
        '''
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(0.1)
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -3*pi/4
        joint_goal[4] = 0
        joint_goal[5] = pi/2
        joint_goal[6] = pi/4 + robotiq * pi/4 # If robotiq 2f-140 is on, go to pi/2, else if it is franka gripper, go to pi/4.
        group.go(joint_goal, wait=True)
        group.stop()
        current_joints = self.group.get_current_joint_values()
        return self.all_joint_close(joint_goal, current_joints, 0.005)

    def go_to_joint_pose(self, joint_target):
        '''
        Plans and execute a trajectory based on a joint goal.
        '''
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(0.1)
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = joint_target[0]
        joint_goal[1] = joint_target[1]
        joint_goal[2] = joint_target[2]
        joint_goal[3] = joint_target[3]
        joint_goal[4] = joint_target[4]
        joint_goal[5] = joint_target[5]
        joint_goal[6] = joint_target[6]
        group.go(joint_goal, wait=True)
        group.stop()
        current_joints = self.group.get_current_joint_values()
        return self.all_joint_close(joint_goal, current_joints, 0.005)

    def go_to_cartesian_pose(self, cartesian_position, quat):
        '''
        Plans and execute a trajectory based on a cartesian goal.
        '''
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(0.1)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = quat[0] 
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        pose_goal.position.x = cartesian_position[0]
        pose_goal.position.y = cartesian_position[1]
        pose_goal.position.z = cartesian_position[2]
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        current_pose = self.group.get_current_pose().pose
        #print(self.group.get_current_joint_values())
        return self.all_cartesian_close(pose_goal, current_pose, 0.005)

    def cartesian_plan_save_execute(self, speed, cartesian_position, quat, plan_file):
        '''
        Plans, SAVE, then execute a trajectory based on a cartesian goal.
        '''
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(speed)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]
        pose_goal.position.x = cartesian_position[0]
        pose_goal.position.y = cartesian_position[1]
        pose_goal.position.z = cartesian_position[2]
        group.set_pose_target(pose_goal)
        to_save = group.plan()
        with open(plan_file, 'w') as file_save:
            yaml.dump(to_save, file_save, default_flow_style=True)
        cur_time = time.time()
        group.execute(to_save, wait=True)
        print("Time taken: ",time.time()-cur_time)
        group.stop()
        group.clear_pose_targets()
        current_pose = self.group.get_current_pose().pose
        #print(self.group.get_current_joint_values())
        return self.all_cartesian_close(pose_goal, current_pose, 0.005)

    def joint_plan_save_execute(self, speed, joint_target, plan_file):
        '''
        Plans, SAVE, then execute a trajectory based on a joint goal.
        '''
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(speed)
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = joint_target[0]
        joint_goal[1] = joint_target[1]
        joint_goal[2] = joint_target[2]
        joint_goal[3] = joint_target[3]
        joint_goal[4] = joint_target[4]
        joint_goal[5] = joint_target[5]
        joint_goal[6] = joint_target[6]
        to_save = group.plan(joint_goal)
        with open(plan_file, 'w') as file_save:
            yaml.dump(to_save, file_save, default_flow_style=True)
        group.execute(to_save, wait=True)
        group.stop()
        group.clear_pose_targets()
        current_joints = self.group.get_current_joint_values()
        return self.all_joint_close(joint_goal, current_joints, 0.005)

    def waypoints_save_execute(self, speed, plan_file):
        '''
        Not fully tested, speed can be quite fast.
        '''
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(speed)
        waypoints = []

        wpose = group.get_current_pose().pose

        for i in range(1):

            wpose.position.z += 0.1
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.y -= 0.1  # Third move sideways (y)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.z -= 0.1  # Third move sideways (y)
            waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (to_save, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

        with open(plan_file, 'w') as file_save:
            yaml.dump(to_save, file_save, default_flow_style=True)
        cur_time = time.time()
        group.execute(to_save, wait=True)
        print("Time taken: ",time.time()-cur_time)
        group.stop()
        #print(self.group.get_current_joint_values())
        return True

    def load_plan_execute(self, plan_file):
        '''
        Load a saved trajectory from joint_plan_save_execute() or cartesian_plan_save_execute() and executes it. 
        '''
        group = self.group
        with open(plan_file, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        group.execute(loaded_plan, wait=True)
        group.stop()
        group.clear_pose_targets()
        return True

    def robotiq_move(self, pos = 0, speed = 0, force = 0):
        '''
        Robotiq commands are non-blocking, and also assumes gripper is reset + activated
        '''
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = pos #only difference between open and close lies here
        cmd.rSP = speed
        cmd.rFR = force
        self.rbtq_pub.publish(cmd)
        return True

    def move_gripper(self, width, speed, wait_result):
        '''
        Calls the Franka Gripper's Move API at;
        https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
        '''
        self.aclient = actionlib.SimpleActionClient('franka_gripper/move', fg.MoveAction)
        self.aclient.wait_for_server()
        gripper_goal = fg.MoveGoal()
        gripper_goal.speed = speed
        gripper_goal.width = width
        self.aclient.send_goal(gripper_goal)
        if wait_result:
            self.aclient.wait_for_result()
            return self.aclient.get_result() ##### returns when move action is complete
        else:
            return True

    def grasp_gripper(self, width, speed, force, wait_result):
        '''
        Calls the Franka Gripper's Grasp API at;
        https://frankaemika.github.io/docs/franka_ros.html#franka-gripper
        Note that we default epsilon.inner = epsilon.outer = 0.1
        '''
        self.aclient = actionlib.SimpleActionClient('franka_gripper/grasp', fg.GraspAction)
        self.aclient.wait_for_server()
        gripper_goal = fg.GraspGoal()
        gripper_goal.speed = speed
        gripper_goal.width = width
        gripper_goal.force = force
        # Following epsilon parameter will fail the grasp if width is not within (desired - inner, desired + outer). We choose epsilon such that all grasps will be successful.
        gripper_goal.epsilon.inner = 0.1 #arbitrary large
        gripper_goal.epsilon.outer = 0.1 #arbitrary large
        self.aclient.send_goal(gripper_goal)
        if wait_result:
            self.aclient.wait_for_result()
            return self.aclient.get_result() ##### returns when grasp is at equilibrium position
        else:
            return True

    def controller_start(self):
        '''
        Ignore this unless you intend to launch controllers from this script.
        Cannot be concurrently running with moveit which also launches its own controller.
        '''
        print("Controller start: ", self.controller_service(['joint_position_example_controller'], [], 2))

    def controller_end(self):
        '''
        Ignore this unless you intend to launch controllers from this script.
        Cannot be concurrently running with moveit which also launches its own controller.
        '''
        print("Controller end: ", self.controller_service([], ['joint_position_example_controller'], 2))

    def filename_publish(self, data):
        '''
        This is for start/stopping recordings.
        '''
        self.filename_pub.publish(data)

    def routine_lock(self):
        '''
        When a routine is running, further requests must be preventing from triggering robot routines.
        '''
        self.lock = True

    def routine_unlock(self):
        '''
        Do this when robot is stationary and completed the routine.
        '''
        self.lock = False

    def play_sound(self, which = 1):
        '''
        Mainly for playing non-blocking sounds, will be useful.
        '''
        pygame.mixer.stop()
        pygame.mixer.music.load(self.sound_directories[which])
        pygame.mixer.music.play(0)

    def something_wrong(self):
        '''
        Stop recording, play warning sound.
        '''
        print("Something went wrong, playing sound and publishing 'stop'.")
        self.play_sound(1)
        self.filename_publish("stop")
        self.routine_unlock()

    def controller_test(self):
        '''
        Ignore this unless you intend to launch controllers from this script.
        Cannot be concurrently running with moveit which also launches its own controller.
        '''
        self.routine_lock()
        self.controller_start()
        rospy.sleep(15)
        self.controller_end()
        self.routine_unlock()

    def home_routine(self):
        '''
        Moves to home position using MoveIt, then reset gripper pose.
        '''
        self.routine_lock()
        if not self.is_moveit_inited:
            self.move_it_init()
        self.go_to_home(robotiq = False)
        #self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957])
        #self.robotiq_move(0,255,1)
        self.move_gripper(0.08, 0.04, True)
        self.routine_unlock()

    def trajectory_saver(self, data):
        '''
        Example trajectory saving routine.
        Trajectory is fully dependent on current joint state.
        In the following example, both trajectorys' start joint state is the home position.
        '''
        self.routine_lock()
        if not self.is_moveit_inited:
            self.move_it_init()

        self.go_to_home(robotiq = False)
        self.cartesian_plan_save_execute(0.1, [0.55,0,0.415],[0.5**0.5, -(0.5**0.5),0,0],'testing_cart.yaml')
        self.go_to_home(robotiq = False)
        self.joint_plan_save_execute(0.1,[0,0.155528,0,-2.1929,0,2.34483,1.56957],'testing_joint.yaml')
        self.go_to_home(robotiq = False)

        self.routine_unlock()

    def data_collection_routine(self, data):
        '''
        Example execution/data-collection routine.
        Trajectory is fully dependent on current joint state.
        In the following example, both trajectorys' start joint state is the home position.
        '''
        self.routine_lock()
        if not self.is_moveit_inited:
            self.move_it_init()

        self.play_sound(0)

        # Start Robotiq Gripper
        for i in range(20):
            self.filename_publish("sept13_trial3_dell_"+str(i))
            self.robotiq_move(200, 0, 0)
            rospy.sleep(6)
            self.robotiq_move(0, 0, 0)
            rospy.sleep(1.5)
            self.filename_publish("stop")
            rospy.sleep(2.5)
        # End Robotiq Gripper

        self.play_sound(0)
        self.routine_unlock()

        return

    def move_to_z(self, z):
        #For testing purposes
        if not self.is_moveit_inited:
            self.move_it_init()
        height=z
        if height >= 0.260 and height <= 0.7: # 0.247
            self.go_to_cartesian_pose([0.55,0,height],[0.5**0.5, -(0.5**0.5),0,0])
        else:
            print("DANGEROUS")


    def tactile_data_collection(self):

        self.routine_lock()
        if not self.is_moveit_inited:
            self.move_it_init()

        # robot going home
        print('robot going home')
        self.go_to_home()

        q1 = [0.707109, -0.707103, 0.0, 0.000747]
        cp1 = [0.307159457837, 7.87796988834e-05, 0.589892090993]
        #self.go_to_cartesian_pose(cp1, q1)

        # calibrate biotac for some time
        self.cmd_sub('Calibrate')
        rospy.sleep(6)

        #print(self.group.get_current_pose().pose)
        q2 = [0.5**0.5, -0.5**0.5, 0.0, 0.0]
        cp2 = [0.507159457837, 0.0, 0.289892090993]
        self.go_to_cartesian_pose(cp2, q2)
    

        self.routine_unlock()

    def cmdCallback(self, data):
        print("I heard: ",data.data)
        if self.lock:
            print("LOCKED, ONGOING ROUTINE!")
        # elif data.data == "controller_test":
        #     self.controller_test()
        elif data.data == "home":
            self.home_routine()
        elif data.data == "traj_save":
            self.trajectory_saver(data) #publish arbitrary string to nake robot execute and save a path
        else:
            self.tactile_data_collection()

def main():
    m_g = MoveGrasp()
    rospy.spin()

if __name__ == '__main__':
    main()
