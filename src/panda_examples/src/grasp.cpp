// This file is full of bad programming practices -- weicong

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <stdlib.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>

#include <cstdio>
double Arr[10];
bool found_cluster = false;
void clbk(const std_msgs::Float32MultiArray::ConstPtr& array) {
    //ROS_INFO("%d", msg->another_field);
  // print all the remaining numbers
  int i = 0;
  for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
    Arr[i] = *it;
    i++;
  }
  ROS_INFO("Received Target");
  found_cluster = true;
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "haf_grasper");
  ros::NodeHandle n;
  ros::Publisher filename_pub = n.advertise<std_msgs::String>("filename", 1000);
  ros::Publisher calib_pub = n.advertise<std_msgs::String>("biotac_sub", 1000);
  ros::Subscriber sub = n.subscribe("/target", 1, clbk);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // const std::string plannerStr = "RRTstarkConfigDefault";
  const std::string plannerStr = "RRTConnectkConfigDefault";
  // const std::string plannerStr = "PRMkConfigDefault";

  std::map<std::string, std::string> plannerParams = move_group.getPlannerParams(plannerStr, PLANNING_GROUP);
  std::cout << "Default parameters" << std::endl;
  std::cout << "==================" << std::endl;
  for (auto it : plannerParams){
    std::cout << it.first << ": " << it.second << std::endl;
  }
  plannerParams["range"] = "0.001";
  move_group.setPlannerParams(plannerStr, PLANNING_GROUP, plannerParams, true);

  // plannerParams = move_group.getPlannerParams(plannerStr, PLANNING_GROUP);
  // std::cout << "New parameters" << std::endl;
  // std::cout << "==================" << std::endl;
  // for (auto it : plannerParams){
  //   std::cout << it.first << ": " << it.second << std::endl;
  // }
  move_group.setPlannerId(plannerStr);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  ROS_INFO_STREAM("here 1");




  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  planning_scene::PlanningScene planning_scene(kinematic_model);

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  // Raw pointers are frequently used to refer to the planning group for improved performance.
//  const robot_state::JointModelGroup* joint_model_group =
//      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  ROS_INFO_STREAM("here 2");

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();
  visual_tools.trigger();

// MY PART
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject desk;
  desk.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  desk.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive_table;
  primitive_table.type = primitive_table.BOX;
  primitive_table.dimensions.resize(3);
  primitive_table.dimensions[0] = 0.6;
  primitive_table.dimensions[1] = 0.6;
  primitive_table.dimensions[2] = 0.6;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.0;
  table_pose.position.y = 0.0;
  table_pose.position.z = -0.3;

  desk.primitives.push_back(primitive_table);
  desk.primitive_poses.push_back(table_pose);
  desk.operation = desk.ADD;

  std::vector<moveit_msgs::CollisionObject> desks;
  desks.push_back(desk);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add a desk into the world");
  planning_scene_interface.addCollisionObjects(desks);
  
  moveit::core::RobotStatePtr current_state0 = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state0->copyJointGroupPositions(joint_model_group, joint_group_positions);
  for (int i=0; i<joint_group_positions.size(); i++) {
    ROS_INFO_STREAM(i << " - " << joint_group_positions[i]);
  }
  joint_group_positions[0] = 0;
  joint_group_positions[1] = -0.785;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = -2.356;
  joint_group_positions[4] = 0;
  joint_group_positions[5] = 1.571;
  joint_group_positions[6] = 0.785;
  // fingers
  joint_group_positions[7] = 0.02;
  joint_group_positions[8] = 0.02;
  
  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan0;
  bool success0 = (move_group.plan(my_plan0) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Moving to start %s", success0 ? "" : "FAILED");
  //move_group.move();
  move_group.execute(my_plan0);


  // When done with the path constraint be sure to clear it.
  //move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  //move_group.setStartStateToCurrentState();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to Plan");

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  while(true) {
    if (found_cluster){
      geometry_msgs::Pose target_pose1;
      std::vector<float> v = {};
      for(int j = 0; j < 10; j++)
      {
        printf("%.15g, ", Arr[j]);
        v.push_back(Arr[j]);
      }
      
      target_pose1.orientation.x = v[6];
      target_pose1.orientation.y = v[7];
      target_pose1.orientation.z = v[8];
      target_pose1.orientation.w = v[9];
      target_pose1.position.x = v[0];//x; //0.28;
      target_pose1.position.y = v[1];//y; //-0.2;
      target_pose1.position.z = v[2];//z; //0.5;
      move_group.setPoseTarget(target_pose1);

      // Now, we call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking move_group
      // to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;

      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("tutorial", "Visualizing Plan (pose goal) %s", success ? "" : "FAILED");

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to run trajectory");
      move_group.execute(my_plan);
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to Plan (again)");


      geometry_msgs::Pose target_pose2;
      
      target_pose2.orientation.x = v[6];
      target_pose2.orientation.y = v[7];
      target_pose2.orientation.z = v[8];
      target_pose2.orientation.w = v[9];
      target_pose2.position.x = v[3];//x; //0.28;
      target_pose2.position.y = v[4];//y; //-0.2;
      target_pose2.position.z = v[5];//z; //0.5;
      move_group.setPoseTarget(target_pose2);

      // Now, we call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking move_group
      // to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

      bool success2 = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("tutorial", "Visualizing Plan (pose goal) %s", success2 ? "" : "FAILED");
      
      //Calibrate here
      std_msgs::String msg;
      std::stringstream ss;
      ss << "calibrate";
      msg.data = ss.str();
      calib_pub.publish(msg);

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to run trajectory");
      ROS_INFO_STREAM("START");
      std_msgs::String msg2;
      std::stringstream ss2;
      ss2 << "dog";
      msg2.data = ss2.str();
      filename_pub.publish(msg2);
      move_group.execute(my_plan2);

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to grip");

      actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp", true);
      ROS_INFO("Waiting for action server to start.");
      bool success3=ac.isServerConnected(); //will wait for infinite time
      if (success3) 
      {
        ROS_INFO("Starting ...");
      }
      ac.waitForServer(); 
      ROS_INFO("Action server started, sending goal."); 
      franka_gripper::GraspGoal goal;
      goal.width=0.0;
      goal.speed=0.01;
      goal.force=10.0;
      //goal.speed=0.01;
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

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to Plan to move up while holding object");

      geometry_msgs::Pose target_pose3;
      
      target_pose3.orientation.x = v[6];
      target_pose3.orientation.y = v[7];
      target_pose3.orientation.z = v[8];
      target_pose3.orientation.w = v[9];
      target_pose3.position.x = v[0];//x; //0.28;
      target_pose3.position.y = v[1];//y; //-0.2;
      target_pose3.position.z = v[2];//z; //0.5;
      move_group.setPoseTarget(target_pose3);

      // Now, we call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking move_group
      // to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan3;

      bool success4 = (move_group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("tutorial", "Visualizing Plan (pose goal) %s", success4 ? "" : "FAILED");

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to run trajectory");
      move_group.execute(my_plan3);

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to Plan to move down while holding object");

      geometry_msgs::Pose target_pose4;
      
      target_pose4.orientation.x = v[6];
      target_pose4.orientation.y = v[7];
      target_pose4.orientation.z = v[8];
      target_pose4.orientation.w = v[9];
      target_pose4.position.x = v[3];//x; //0.28;
      target_pose4.position.y = v[4];//y; //-0.2;
      target_pose4.position.z = v[5];//z; //0.5;
      move_group.setPoseTarget(target_pose4);

      // Now, we call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking move_group
      // to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan4;

      bool success5 = (move_group.plan(my_plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("tutorial", "Visualizing Plan (pose goal) %s", success5 ? "" : "FAILED");

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to run trajectory");
      move_group.execute(my_plan4);

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to Release Gripper");
      actionlib::SimpleActionClient<franka_gripper::HomingAction> ac2("franka_gripper/homing", true);
      ROS_INFO("Waiting for action server to start.");
      bool success6=ac2.isServerConnected(); //will wait for infinite time
      if (success6) 
      {
        ROS_INFO("Starting ...");
      }
      ac2.waitForServer(); 
      ROS_INFO("Action server started, sending goal."); 
      franka_gripper::HomingGoal goal2;

      ac2.sendGoal(goal2);

      //wait for the action to return
      bool finished_before_timeout2 = ac2.waitForResult(ros::Duration(30.0));

      if (finished_before_timeout2)
      {
        actionlib::SimpleClientGoalState state = ac2.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
      }
      else
        ROS_INFO("Action did not finish before the time out.");

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to Plan to move up");

      geometry_msgs::Pose target_pose5;
      
      target_pose5.orientation.x = v[6];
      target_pose5.orientation.y = v[7];
      target_pose5.orientation.z = v[8];
      target_pose5.orientation.w = v[9];
      target_pose5.position.x = v[0];//x; //0.28;
      target_pose5.position.y = v[1];//y; //-0.2;
      target_pose5.position.z = v[2];//z; //0.5;
      move_group.setPoseTarget(target_pose5);

      // Now, we call the planner to compute the plan and visualize it.
      // Note that we are just planning, not asking move_group
      // to actually move the robot.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan5;

      bool success7 = (move_group.plan(my_plan5) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      ROS_INFO_NAMED("tutorial", "Visualizing Plan (pose goal) %s", success7 ? "" : "FAILED");

      ROS_INFO_STREAM("END");
      std_msgs::String msg3;
      std::stringstream ss3;
      ss3 << "stop";
      msg3.data = ss3.str();
      filename_pub.publish(msg3);

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to run trajectory");
      move_group.execute(my_plan5);

      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to go back to start");

      move_group.setJointValueTarget(joint_group_positions);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan6;
      bool success8 = (move_group.plan(my_plan6) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Moving to start %s", success8 ? "" : "FAILED");
      //move_group.move();
      move_group.execute(my_plan6);
      found_cluster = false;
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to try next grasp");
    }
    else {
      ROS_INFO("No target registered");
      ros::shutdown();
      return 0;
    }
  }

}
