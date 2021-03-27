#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
//#include <franka_gripper/HomingAction.h>
//#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>

#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka/gripper.h>

int main(int argc, char** argv) {
  if (argc != 6) {
    std::cerr << "Usage: ./grasp_object <gripper-hostname> <homing> <object-width> <speed> <force>" << std::endl;
    return -1;
  }

  try {
    franka::Gripper gripper(argv[1]);
    double grasping_width = std::stod(argv[3]);
    double epsilon_inner = 1.0; //default is 0.005
    double epsilon_outer = 1.0; //default is 0.005

    std::stringstream ss(argv[2]);
    bool homing;
    if (!(ss >> homing)) {
      std::cerr << "<homing> can be 0 or 1." << std::endl;
      return -1;
    }

    if (homing) {
      // Do a homing in order to estimate the maximum grasping width with the current fingers.
      gripper.homing();
    }

    // Check for the maximum grasping width.
    franka::GripperState gripper_state = gripper.readOnce();
    if (gripper_state.max_width < grasping_width) {
      std::cout << "Object is too large for the current fingers on the gripper." << std::endl;
      return -1;
    }

    // Grasp the object.
    if (!gripper.grasp(grasping_width, std::stod(argv[4]), std::stod(argv[5]), epsilon_inner, epsilon_outer)) {
      std::cout << "Failed to grasp object." << std::endl;
      return -1;
    }

    else {
      std::cout << "Gripped." << std::endl;
    }

    // Wait 3s and check afterwards, if the object is still grasped.
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(5000));

    gripper_state = gripper.readOnce();
    if (!gripper_state.is_grasped) {
      std::cout << "Object lost." << std::endl;
      return -1;
    }

    std::cout << "Grasped object, will release it now." << std::endl;
    gripper.stop();
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}

// https://answers.ros.org/question/290053/passing-arguments-with-action-client/
