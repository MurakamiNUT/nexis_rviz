/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Sachin Chitta */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// Kinematics
#include <moveit_msgs/GetPositionIK.h>

#include <sensor_msgs/JointState.h>
#include "pc_side_programs/Controller.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
// void rotation_matrix_calc();

#define PI 3.1415926535
// using namespace std;
using Eigen::MatrixXd;

// moveit_msgs::DisplayRobotState msg;
// ros::Publisher robot_state_publisher;
void publish_robot_state();

bool pose_reset = false;
int32_t key;
void chatterCallback_key(const std_msgs::Int32& key_string){
  key = key_string.data;
  ROS_INFO("data:[%d]", key);
  switch(key){
    case 97://a
      pose_reset = true;
    break;
  }
}
int main(int argc, char** argv){
  ros::init(argc, argv, "arm_control");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);

  ros::Subscriber sub_key = node_handle.subscribe("key", 1000, chatterCallback_key);
  ros::Publisher robot_state_publisher =
      node_handle.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);

  moveit_msgs::DisplayRobotState msg;
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("r5s_arm");


  msg.state.joint_state.name.resize(6);
  msg.state.joint_state.name[0] = "body1_joint";
  msg.state.joint_state.name[1] = "body2_joint";
  msg.state.joint_state.name[2] = "body3_joint";
  msg.state.joint_state.name[3] = "body4_joint";
  msg.state.joint_state.name[4] = "body5_joint";
  msg.state.joint_state.name[5] = "body6_joint";

  msg.state.joint_state.position.resize(6);
  for(int i=0;i < 6;i++){
    msg.state.joint_state.position[i] = 0.;
  }

  ros::Duration(2.0).sleep();

  while(ros::ok()){
    if(pose_reset){
      robot_state_publisher.publish(msg);
      pose_reset = false;
    }
    ros::spinOnce();//spinはアイドルループかも?
    loop_rate.sleep();
  }
}