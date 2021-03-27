// Known bugs:
// 1. If gripper_grasp does not grasp within width +- episilon, it does not apply force rightfully, but it does not return any exception.

//#include <iostream>
//#include <cstdio>
//#include <stdlib.h>
//#include <actionlib/client/terminal_state.h>
//#include <franka/exception.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka/gripper.h>
#include <thread>
#include "std_msgs/String.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"


#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager/controller_manager.h> //Added by weicong
#include <boost/shared_ptr.hpp> //Added by weicong
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std;
 
class MoveGrasp {
  public:
    ros::NodeHandle n; 
    ros::Publisher calib_pub;
    ros::Publisher filename_pub;
    ros::Publisher robotiq_command_pub;
    ros::Subscriber sub;
    ros::ServiceClient switch_client;
    std::fstream fout;
    void set_target_callback(const std_msgs::String::ConstPtr& msg);
    void calibrate_biotac(int duration);
    void sleep(double duration);
    void filename_publish(std::string filename);
    void joint_moveit_move(std::vector<double> joint_group_positions);
    void cartesian_moveit_move(double velocity_scale, double pos_x, double pos_y, double pos_z, double quat_x, double quat_y, double quat_z, double quat_w);
    void gripper_grasp(int duration, double width, double speed, double force, double eps_inner, double eps_outer);
    void gripper_move(double width, double speed);
    void robotiq_open(int speed, int force);
    void robotiq_close(int speed, int force);
    void controller_start();
    void controller_end();
    void routine_1(const std_msgs::String::ConstPtr& msg);
    void routine_2(const std_msgs::String::ConstPtr& msg);
    void routine_3(const std_msgs::String::ConstPtr& msg);
    void routine_4(const std_msgs::String::ConstPtr& msg);
    void routine_5(const std_msgs::String::ConstPtr& msg);
    void routine_6(const std_msgs::String::ConstPtr& msg);
    void routine_7(const std_msgs::String::ConstPtr& msg);
    void routine_8(const std_msgs::String::ConstPtr& msg);
    void routine_9(const std_msgs::String::ConstPtr& msg);
    void gripper_example_routine(const std_msgs::String::ConstPtr& msg);
    MoveGrasp();
};
 
MoveGrasp::MoveGrasp(void) {
  ros::NodeHandle n;
  filename_pub = n.advertise<std_msgs::String>("filename", 1);
  calib_pub = n.advertise<std_msgs::String>("biotac_sub", 1);
  robotiq_command_pub = n.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/Robotiq2FGripperRobotOutput", 1);
  sub = n.subscribe("/target", 1, &MoveGrasp::set_target_callback, this);
  switch_client = n.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
}

void MoveGrasp::calibrate_biotac(int duration = 2000){
  std_msgs::String msg;
  std::string ss = "calibrate";
  msg.data = ss;
  calib_pub.publish(msg);
  ROS_INFO("Calibrating Biotac");
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(duration));
  ROS_INFO("Calibrated!");
}

void MoveGrasp::filename_publish(std::string filename){
  std_msgs::String msg;
  msg.data = filename;
  filename_pub.publish(msg);
}

void MoveGrasp::joint_moveit_move( std::vector<double> joint_group_positions = {0, -0.785, 0, -2.356, 0, 1.571, 0.785}) {
  std::string PLANNING_GROUP = "panda_arm";
  std::string plannerStr = "RRTConnectkConfigDefault";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  std::map<std::string, std::string> plannerParams = move_group.getPlannerParams(plannerStr, PLANNING_GROUP);
  move_group.setPlannerParams(plannerStr, PLANNING_GROUP, plannerParams, true);
  move_group.setPlannerId(plannerStr);

  move_group.setJointValueTarget(joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //Start
  // ROS_INFO("OUTPUT NOW 1");
  // YAML::Node node = YAML::Load("[1, 2, 3]");
  // std::string output_filename = "my_plan.yaml";
  // fout.open(output_filename, std::fstream::out | std::fstream::app);
  // //fout << "whatever\n";
  // fout << node;
  // fout.close();
  // ROS_INFO("OUTPUT NOW 2");
  //End
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group.execute(my_plan);
}

void MoveGrasp::cartesian_moveit_move( double velocity_scale = 1.0, double pos_x = 0.307382106728, double pos_y = 0.0, double pos_z = 0.588749153957, double quat_x = 0.92416610616, double quat_y = -0.381986892882, double quat_z = -0.000972960439309, double quat_w = 0.00144056942286) {
  //For Robotiq, the home position's quaternion is 0.707106781, -0.707106781, 0, 0.
  std::string PLANNING_GROUP = "panda_arm";
  std::string plannerStr = "RRTConnectkConfigDefault";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setMaxVelocityScalingFactor(velocity_scale); // VELOCITY
  move_group.setNumPlanningAttempts(10);
  move_group.setPlanningTime(5);
  std::map<std::string, std::string> plannerParams = move_group.getPlannerParams(plannerStr, PLANNING_GROUP);
  move_group.setPlannerParams(plannerStr, PLANNING_GROUP, plannerParams, true);
  move_group.setPlannerId(plannerStr);
  move_group.setGoalTolerance(0.001);

  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = quat_x;
  target_pose.orientation.y = quat_y;
  target_pose.orientation.z = quat_z;
  target_pose.orientation.w = quat_w;
  target_pose.position.x = pos_x;//x; //0.28;
  target_pose.position.y = pos_y;//y; //-0.2;
  target_pose.position.z = pos_z;//z; //0.5;
  move_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  std::cout << "Planning time: " << my_plan.planning_time_ << std::endl;
  move_group.execute(my_plan);
}

void MoveGrasp::gripper_grasp(int duration = 3000, double width=0.01, double speed=0.01, double force=30, double eps_inner=0.1, double eps_outer=0.1) {
  ROS_INFO("Grasp start");
  actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp("franka_gripper/grasp", true);
  bool success_grasp=ac_grasp.isServerConnected();
  ac_grasp.waitForServer(); 
  franka_gripper::GraspGoal goal_grasp;
  goal_grasp.width=width;
  goal_grasp.speed=speed;
  goal_grasp.force=force;
  goal_grasp.epsilon.inner=eps_inner;
  goal_grasp.epsilon.outer=eps_outer;
  ac_grasp.sendGoal(goal_grasp);
  bool finished_before_timeout = ac_grasp.waitForResult(ros::Duration(30.0));
  ROS_INFO("Grasp complete");
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(duration));
}

void MoveGrasp::gripper_move(double width=0.08, double speed=0.01) {
  ROS_INFO("Move start");
  actionlib::SimpleActionClient<franka_gripper::MoveAction> ac("franka_gripper/move", true);
  bool success=ac.isServerConnected();
  ac.waitForServer(); 
  franka_gripper::MoveGoal goal;
  goal.width=width;
  goal.speed=speed;
  ac.sendGoal(goal);
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  ROS_INFO("Move complete");
}


//Robotiq gripper commands are NON-BLOCKING, and assumes gripper is reset+activated.
void MoveGrasp::robotiq_open(int speed=0, int force=0){
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msg;
  msg.rACT = 1;
  msg.rGTO = 1;
  msg.rSP = speed;
  msg.rFR = force;
  msg.rPR = 0;
  robotiq_command_pub.publish(msg);
}

void MoveGrasp::robotiq_close(int speed=0, int force=0){
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output msg;
  msg.rACT = 1;
  msg.rGTO = 1;
  msg.rSP = speed;
  msg.rFR = force;
  msg.rPR = 255;
  robotiq_command_pub.publish(msg);
}

void MoveGrasp::controller_start(){
  // std::vector < std::string > start_controllers;
  // std::vector < std::string > stop_controllers;
  // start_controllers.push_back("cartesian_ptp_controller");
  // stop_controllers.push_back("");
  // if (!controller_manager_->switchController(start_controllers, stop_controllers, controller_manager_msgs::SwitchController::Request::STRICT)) {
  //   ROS_ERROR_STREAM_NAMED("cartesian_ptp_controller","Failed to switch controllers");
  // } 
  // else {
  //   ROS_INFO("Controller started.");
  // }

  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers.push_back("cartesian_ptp_controller");
  srv.request.stop_controllers.push_back("");
  srv.request.strictness = srv.request.BEST_EFFORT;
  bool call_success = switch_client.call(srv);
}

void MoveGrasp::controller_end(){
  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers.push_back("");
  srv.request.stop_controllers.push_back("cartesian_ptp_controller");
  srv.request.strictness = srv.request.BEST_EFFORT;
  bool call_success = switch_client.call(srv);
}

void MoveGrasp::sleep(double duration = 3000)
{
  ROS_INFO("Sleep start");
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(duration));
  ROS_INFO("Sleep complete");
}

void MoveGrasp::set_target_callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s], target received.", msg->data.c_str());
  if (msg->data == "eg") {
    ROS_INFO("gripper_example_routine");
    MoveGrasp::gripper_example_routine(msg);
  }
  else if (msg->data == "two"){
    ROS_INFO("2");
    MoveGrasp::routine_2(msg);
  }
  else {
    ROS_INFO("Collect data");
    MoveGrasp::routine_1(msg);
  }
}

void MoveGrasp::routine_1(const std_msgs::String::ConstPtr& msg)
{
	//MoveGrasp::sleep(1000);
  MoveGrasp::joint_moveit_move({-0.000595043,0.48909,0.00020022,-1.44323,-0.000737282,1.92488,1.57179});
  MoveGrasp::filename_publish(msg->data);
	MoveGrasp::cartesian_moveit_move(1.0, 0.7, 0.1, 0.4, 0.707106781, -0.707106781, 0, 0);
  MoveGrasp::cartesian_moveit_move(1.0, 0.7, -0.1, 0.4, 0.707106781, -0.707106781, 0, 0);
  MoveGrasp::cartesian_moveit_move(1.0, 0.7, 0.1, 0.4, 0.707106781, -0.707106781, 0, 0);
  MoveGrasp::cartesian_moveit_move(1.0, 0.7, -0.1, 0.4, 0.707106781, -0.707106781, 0, 0);
  MoveGrasp::filename_publish("stop"); 
  MoveGrasp::sleep(2000);
  MoveGrasp::joint_moveit_move({0, -0.785, 0, -2.356, 0, 1.571, 1.571});
}

void MoveGrasp::routine_2(const std_msgs::String::ConstPtr& msg)
{
  MoveGrasp::controller_start();
  MoveGrasp::sleep(15000);
  MoveGrasp::controller_end();
}

void MoveGrasp::routine_3(const std_msgs::String::ConstPtr& msg)
{
}

void MoveGrasp::routine_4(const std_msgs::String::ConstPtr& msg)
{
}

void MoveGrasp::routine_5(const std_msgs::String::ConstPtr& msg)
{
}

void MoveGrasp::routine_6(const std_msgs::String::ConstPtr& msg)
{
}

void MoveGrasp::routine_7(const std_msgs::String::ConstPtr& msg)
{
}

void MoveGrasp::routine_8(const std_msgs::String::ConstPtr& msg)
{
}

void MoveGrasp::routine_9(const std_msgs::String::ConstPtr& msg)
{
}

void MoveGrasp::gripper_example_routine(const std_msgs::String::ConstPtr& msg)
{
  MoveGrasp::filename_publish(msg->data); //Start recording data
  MoveGrasp::sleep(1000);
  MoveGrasp::joint_moveit_move({-1.571, -0.785, 0, -2.356, 0, 1.571, 0.785});
  //MoveGrasp::calibrate_biotac();
  //MoveGrasp::gripper_move(0.05, 0.05); // Move has no notion of duration, and therefore you need to call sleep to keep the gripper state for n seconds.
  MoveGrasp::sleep(3000);
  // MoveGrasp::gripper_move(0.08, 0.05);
  // MoveGrasp::sleep(1000);
  // MoveGrasp::gripper_grasp(2000); // Grasp already has in-built grasp duration, therefore we can just set duration = 2000 to grasp for 2 seconds.
  // MoveGrasp::gripper_move();
  // MoveGrasp::sleep(1000);
  MoveGrasp::cartesian_moveit_move();
  MoveGrasp::sleep(3000);
  MoveGrasp::joint_moveit_move();
  MoveGrasp::sleep(1000);
  MoveGrasp::robotiq_close();
  MoveGrasp::sleep(3000);
  MoveGrasp::robotiq_open();
  MoveGrasp::filename_publish("stop"); //Stop recording data
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "move_grasp");
  MoveGrasp m_g;
  ros::AsyncSpinner spinner(2); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}