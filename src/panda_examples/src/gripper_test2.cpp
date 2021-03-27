#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka/gripper.h>

int main (int argc, char **argv)
{

   ros::init(argc, argv, "gripper_test1");
   if(argc != 4) {
       ROS_INFO("error!");
       return 1;
   }

   actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp", true);
   actionlib::SimpleActionClient<franka_gripper::MoveAction> ac2("franka_gripper/move", true);
   //actionlib::SimpleActionClient<franka_gripper::HomingAction> ac("franka_gripper/homing", true);
   ROS_INFO("Waiting for action server to start.");
   bool success=ac.isServerConnected(); //will wait for infinite time
   if (success) 
   {
      ROS_INFO("Starting ...");
   }
   ac.waitForServer(); 
   ROS_INFO("Action server started, sending goal."); 
   franka_gripper::GraspGoal goal;
   goal.width=atof(argv[1]);
   goal.speed=atof(argv[2]);
   goal.force=atof(argv[3]);
   goal.epsilon.inner=0.1;
   goal.epsilon.outer=0.1;
   ac.sendGoal(goal);
   //franka_gripper::HomingGoal goal;
   //ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(5000));
  ROS_INFO("Waiting for action server to start.");
  bool success2=ac2.isServerConnected(); //will wait for infinite time
  if (success2) 
  {
    ROS_INFO("Starting ...");
  }
  ac2.waitForServer(); 
  ROS_INFO("Action server started, sending goal."); 
  franka_gripper::MoveGoal goal2;
  goal2.width=0.08;
  goal2.speed=0.01;
  ac2.sendGoal(goal2);
  //exit
  //std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(5000));

  return 0;
}

// https://answers.ros.org/question/290053/passing-arguments-with-action-client/
