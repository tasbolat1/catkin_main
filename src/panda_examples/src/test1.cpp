/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "std_msgs/Float32MultiArray.h"
#include <stdlib.h>

#include <cstdio>
double Arr[3];
void clbk(const std_msgs::Float32MultiArray::ConstPtr& array) {
    //ROS_INFO("%d", msg->another_field);
  // print all the remaining numbers
  int i = 0;
  for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
    Arr[i] = *it;
    i++;
  }
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");

  if(argc != 8	) {
      ROS_INFO("Not enough arguments");
      ros::shutdown();
      return 0;
  }
  double x = atof(argv[1]);
  double y = atof(argv[2]);
  double z = atof(argv[3]);

  double q0 = atof(argv[4]);
  double q1 = atof(argv[5]);
  double q2 = atof(argv[6]);
  double q3 = atof(argv[7]);

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/array", 1, clbk);


  ROS_INFO("PLanning to reach %g %g %g", x, y, z);
  ros::NodeHandle node_handle;
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
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success0 ? "" : "FAILED");
  //move_group.move();
  move_group.execute(my_plan0);


  // When done with the path constraint be sure to clear it.
  //move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  //move_group.setStartStateToCurrentState();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    for(int j = 0; j < 3; j++)
    {
      printf("%.15g, ", Arr[j]);
    }
    target_pose1.orientation.w = q0;
    target_pose1.orientation.x = q1;
    target_pose1.orientation.y = q2;
    target_pose1.orientation.z = q3;
    target_pose1.position.x = Arr[0];//x; //0.28;
    target_pose1.position.y = Arr[1];//y; //-0.2;
    target_pose1.position.z = Arr[2]+0.1;//z; //0.5;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to run trajectory");
  move_group.execute(my_plan);

  ros::shutdown();
  return 0;
}
