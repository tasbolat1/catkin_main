#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import yaml
import time
import os
import csv
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output as outputMsg
from moveit_msgs.msg import RobotTrajectory
from controller_manager_msgs.srv import SwitchController
import pygame

#To-do: Run .yaml trajectory with joint_state check, Controller start/end, 
#Low-priority to-do: Franka gripper grasp/move, Biotac calibrate.

class MoveGrasp(object):
    def __init__(self):
        super(MoveGrasp, self).__init__()
        rospy.init_node('move_grasp_python', anonymous=True)
        self.filename_pub = rospy.Publisher('/filename', String, queue_size = 1000) #publishing here saves robotiq+franka+aces+realsense+prophesee data.
        self.tactile_filename_pub = rospy.Publisher('/filename_tactile', String, queue_size = 1000) #publishing here saves only tactile data
        self.rbtq_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg, queue_size = 1000)
        self.target_sub = rospy.Subscriber('/target', String, self.targetCallback, queue_size=1000)
        self.controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

        # Misc variables
        pygame.init()
        pygame.mixer.init()
        self.sound_directories = ['/home/crslab/panda_sim/panda_examples/correct.ogg','/home/crslab/panda_sim/panda_examples/warning.ogg','/home/crslab/panda_sim/panda_examples/try_again.ogg']
        self.lock = False
        self.offset = 0.002 #in m, for 4th movement in data collection, so that item gently or not come into contact with table.
        self.is_moveit_inited = False
        self.lift_height = 0.05 #in m, for 3rd movement in data_collection
        self.f = None
        self.traj_timestamps_holder = list() #must be 6 values at the end.
        with open('/home/crslab/panda_sim/panda_examples/objects_heights.csv') as f:
            reader = csv.reader(f, skipinitialspace=True)
            self.height_dict = dict(reader)
        

    def move_it_init(self):
        if self.is_moveit_inited:
            return
        self.is_moveit_inited = True
        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  In this case the group is the joints in the Panda
        ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
        ## you should change this value to the name of your robot arm planning group.
        ## This interface can be used to plan and execute motions on the Panda:
        group_name = "panda_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""
        ## END_SUB_TUTORIAL

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
                    print(index, "   ",abs(actual[index] - goal[index]))

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
                if index <= 2 and abs(actual[index] - goal[index]) > tolerance: #for oosition
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
                    print(index, "   ",abs(actual[index] - goal[index]))

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_cartesian_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_cartesian_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True


    def go_to_home(self, robotiq = True):
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(0.5)
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
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(0.5)
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
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(0.2)
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
        print(self.group.get_current_joint_values())
        return self.all_cartesian_close(pose_goal, current_pose, 0.005)

    def cartesian_plan_save_execute(self, speed, cartesian_position, quat, plan_file):
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

    def waypoints_save_execute(self, speed, plan_file):
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
        group.set_max_velocity_scaling_factor(speed)
        waypoints = []

        wpose = group.get_current_pose().pose

        for i in range(3):
            wpose.position.x += 0.1
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.z += 0.1
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x -= 0.1  # Third move sideways (y)
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

    def joint_plan_save_execute(self, joint_target, plan_file):
        if not self.is_moveit_inited:
            print("MoveIt not initialized!")
            return
        group = self.group
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

    def load_plan_execute(self, plan_file):
        group = self.group
        with open(plan_file, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        self.timestamp_append()
        group.execute(loaded_plan, wait=True)
        group.stop()
        self.timestamp_append()
        group.clear_pose_targets()
        return True

    #Robotiq commands are non-blocking, and also assumes gripper is reset + activated
    def robotiq_open(self, speed = 0, force = 0):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = 0 #only difference between open and close lies here
        cmd.rSP = speed
        cmd.rFR = force
        self.rbtq_pub.publish(cmd)
        return True

    def robotiq_close(self, speed = 0, force = 0):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = 255 #only difference between open and close lies here
        cmd.rSP = speed
        cmd.rFR = force
        self.rbtq_pub.publish(cmd)
        return True

    def robotiq_move(self, pos = 0, speed = 0, force = 0):
        cmd = outputMsg()
        cmd.rACT = 1
        cmd.rGTO = 1
        cmd.rPR = pos #only difference between open and close lies here
        cmd.rSP = speed
        cmd.rFR = force
        self.rbtq_pub.publish(cmd)
        return True

    def controller_start(self):
        #If panda_moveit.launch is running, most likely whatever controller here will have conflicts.
        print("Controller start: ", self.controller_service(['joint_position_example_controller'], [], 2))

    def controller_end(self):
        print("Controller end: ", self.controller_service([], ['joint_position_example_controller'], 2))

    def filename_publish(self, data):
        self.filename_pub.publish(data)

    def tactile_filename_publish(self, data):
        self.tactile_filename_pub.publish(data)

    def routine_lock(self):
        #When a routine is running, it should be locked away from accepting routine requests
        self.lock = True

    def routine_unlock(self):
        #Do this when robot is stationary and completed the routine.
        self.lock = False

    def timestamp_append(self):
        self.traj_timestamps_holder.append(time.time())

    def list_to_string(self, convert_list):
        for i in range(len(convert_list)):
            if i == 0:
                the_string = str(convert_list[i])
            else:
                the_string += " "
                the_string += str(convert_list[i])
        return the_string

    def pose_check(self, desired_xyz, desired_quat):
        current_pose = self.group.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = desired_quat[0]
        pose_goal.orientation.y = desired_quat[1]
        pose_goal.orientation.z = desired_quat[2]
        pose_goal.orientation.w = desired_quat[3]
        pose_goal.position.x = desired_xyz[0]
        pose_goal.position.y = desired_xyz[1]
        pose_goal.position.z = desired_xyz[2]
        #print(self.group.get_current_joint_values())
        return self.all_cartesian_close(pose_goal, current_pose, 0.005)

    def play_sound(self, which = 1):
        pygame.mixer.stop()
        pygame.mixer.music.load(self.sound_directories[which])
        pygame.mixer.music.play(0)

    def something_wrong(self):
        print("Something went wrong, playing sound and publishing 'stop'.")
        self.play_sound(1)
        self.traj_timestamps_holder = list()
        self.f.close()
        self.filename_publish("stop")
        self.routine_unlock()

    def targetCallback(self, data):
        print("I heard: ",data.data)
        if self.lock:
            print("LOCKED, ONGOING ROUTINE!")
        elif data.data == "controller_test":
            self.controller_test()
        elif data.data == "warmup":
            self.warmup_aces()
        elif data.data == "home":
            self.home_routine()
        elif data.data[0:9] == "intensity":
            self.intensity_referencing(data)
        elif data.data[0:9] == "alignment":
            self.alignment_referencing_plot(data)
        elif data.data[0:7] == "handref":
            self.hand_referencing_plot(data)
        elif data.data[0:5] == "move_":
            self.move_to_obj_height(data)
        else:
            #self.move_to_z(data) #publish 'a260' string to make robot move to z=0.26
            #self.trajectory_saver(data) #publish 'a260' string to make robot prepare and save a path that stops at z=0.26
            self.data_collection_routine(data)

    def controller_test(self):
        self.routine_lock()
        self.controller_start()
        rospy.sleep(15)
        self.controller_end()
        self.routine_unlock()

    def home_routine(self):
        self.routine_lock()
        if not self.is_moveit_inited:
            self.move_it_init()
        #self.go_to_home(robotiq = True)
        self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957])
        self.robotiq_open(255,1)
        self.routine_unlock()

    def move_to_z(self,z):
        #For testing purposes
        if not self.is_moveit_inited:
            self.move_it_init()
        if not isinstance(z, str):
            zstr = z.data
        else:
            zstr = z
        height = int(zstr[1:])/1000.0
        if height >= 0.260 and height <= 0.7: # 0.247
            self.go_to_cartesian_pose([0.55,0,height],[0.5**0.5, -(0.5**0.5),0,0])
        else:
            print("DANGEROUS")

    def warmup_aces(self):
        self.routine_lock()
        for i in range(50):
            self.robotiq_close(100,0)
            rospy.sleep(5) #grip for 2 seconds
            self.robotiq_open(255,1)
            rospy.sleep(1.5)
        self.routine_unlock()

    def intensity_referencing(self, data): #no_move yes_close
        self.routine_lock()
        self.tactile_filename_publish(data)
        self.robotiq_close(0,1) 
        rospy.sleep(5) #grip for 3 seconds
        self.tactile_filename_publish("stop")
        self.robotiq_open(255,1)
        self.routine_unlock()

    def alignment_referencing_plot(self, data): #yes_move yes_close
        self.routine_lock()
        self.move_to_z("a310")
        self.tactile_filename_publish(data)
        self.robotiq_close(0,1)
        rospy.sleep(5.1) #grip for 3 seconds
        self.tactile_filename_publish("stop")
        self.robotiq_open(255,1)
        self.routine_unlock()

    def hand_referencing_plot(self, data): #no_move no_close
        self.tactile_filename_publish(data)
        rospy.sleep(5)
        self.tactile_filename_publish("stop")

    def trajectory_saver(self, z):
        self.routine_lock()
        if not self.is_moveit_inited:
            self.move_it_init()
        self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957]) #0.315 height
        self.cartesian_plan_save_execute(0.01, [0.55,0,0.415],[0.5**0.5, -(0.5**0.5),0,0],'go_up_01.yaml') #18.46
        self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957])
        self.cartesian_plan_save_execute(0.02, [0.55,0,0.415],[0.5**0.5, -(0.5**0.5),0,0],'go_up_02.yaml') #9.25
        self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957])
        self.cartesian_plan_save_execute(0.05, [0.55,0,0.415],[0.5**0.5, -(0.5**0.5),0,0],'go_up_05.yaml') #3.83
        self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957])
        self.cartesian_plan_save_execute(0.1, [0.55,0,0.415],[0.5**0.5, -(0.5**0.5),0,0],'go_up_1.yaml') #1.96
        self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957])
        self.cartesian_plan_save_execute(0.2, [0.55,0,0.415],[0.5**0.5, -(0.5**0.5),0,0],'go_up_2.yaml') #1.13
        self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957])
        self.cartesian_plan_save_execute(0.4, [0.55,0,0.415],[0.5**0.5, -(0.5**0.5),0,0],'go_up_4.yaml') #0.76
        #self.waypoints_save_execute(0.2,'create_slip.yaml')
        self.routine_unlock()

    def move_to_obj_height(self, data):
        try:
            height = int(self.height_dict[data.data[5:]])
            self.play_sound(0)
        except KeyError:
            self.play_sound(2)
            print("Filename not recognized, most likely typo.")
            return
        self.routine_lock()
        if not self.is_moveit_inited:
            self.move_it_init()
        self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957])
        self.load_plan_execute('/home/crslab/franka_traj/v2_fakehome_obj_'+str(height)+'.yaml')
        if not self.pose_check([0.4,0,height/1000.0],[0.5**0.5, -(0.5**0.5),0,0]):
            self.something_wrong()
            return
        self.traj_timestamps_holder = list()
        self.routine_unlock()
        return

    def data_collection_routine(self, data):
        # try:
        #     pos = int(data.data[-6:-3])
        #     self.play_sound(0)
        # except:
        #     self.play_sound(2)
        #     print("Filename not recognized, most likely typo.")
        #     return
        # if pos < 130 or pos > 200:
        #     self.play_sound(2)
        #     print("Filename not recognized, most likely typo.")
        #     return
        pos = 168 #176 training
        stage1_duration = 1.0
        self.routine_lock()
        if not self.is_moveit_inited:
            self.move_it_init()
        if not data.data[0:4] == "nope":
            self.f = open(data.data+".startend", "a")
        self.traj_timestamps_holder = list() #wipe list
        self.go_to_joint_pose([0,0.155528,0,-2.1929,0,2.34483,1.56957])
        self.robotiq_move(pos,150,0)
        rospy.sleep(3.0)
        if not data.data[0:4] == "nope":
            self.filename_publish(data)
        else:
            print("DON'T RECORD")
        self.timestamp_append()
        rospy.sleep(1.0)

        cur_time = time.time()
        self.load_plan_execute('/home/crslab/franka_traj/go_up_4.yaml')
        print('Lift duration:', time.time()-cur_time)
        if cur_time + stage1_duration - time.time() < 0 or not self.pose_check([0.55,0,0.415],[0.5**0.5, -(0.5**0.5),0,0]):
            self.something_wrong()
            return
        rospy.sleep(cur_time + stage1_duration - time.time())

        rospy.sleep(4.0)
        self.play_sound(0)
        if not data.data[0:4] == "nope":
            self.filename_publish("stop")
        self.timestamp_append()


        if len(self.traj_timestamps_holder) != 4:
            print("Timestamp holder incomplete,", self.traj_timestamps_holder)
            self.something_wrong()
            return
        if not data.data[0:4] == "nope":
            self.f.write(self.list_to_string(self.traj_timestamps_holder))
            self.traj_timestamps_holder = list()
            self.f.close()

        self.routine_unlock()
        return


def main():
    m_g = MoveGrasp()
    rospy.spin()

if __name__ == '__main__':
    main()
