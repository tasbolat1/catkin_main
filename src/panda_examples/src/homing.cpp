#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/HomingAction.h>
//#include <franka_gripper/MoveAction.h>
//#include <franka_gripper/GraspAction.h>

#include <cstdio>

int main (int argc, char **argv)
{

   ros::init(argc, argv, "homing");
   
   actionlib::SimpleActionClient<franka_gripper::HomingAction> ac("franka_gripper/homing", true);
   ROS_INFO("Waiting for action server to start.");
   bool success=ac.isServerConnected(); //will wait for infinite time
   if (success) 
   {
      ROS_INFO("Starting ...");
   }
   ac.waitForServer(); 
   ROS_INFO("Action server started, sending goal."); 
   franka_gripper::HomingGoal goal;

   ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

// https://answers.ros.org/question/290053/passing-arguments-with-action-client/
