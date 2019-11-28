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

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "moveit_msgs/DisplayRobotState.h"
#include "moveit_msgs/DisplayTrajectory.h"

#include "moveit_msgs/AttachedCollisionObject.h"
#include "moveit_msgs/CollisionObject.h"

#include "moveit_visual_tools/moveit_visual_tools.h"
#include "pc_side_programs/Controller.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <Eigen/Dense>
#define PI 3.1415926535
using namespace std;
using Eigen::MatrixXd;
using namespace visualization_msgs;
bool START = false;
double Position[3] = {0.12435, 0., 0.20339};//x y z
//オイラー
double Rotation[3] = {0, 0, 0};//roll pitch yaw
//ラジアン
double Rotation_rad[3] = {0, 0, 0};
tf::Quaternion quaternion;
geometry_msgs::Pose t_pose;
tf::Transform transform;
//行列作成
//回転行列
MatrixXd Tmx(3,3),Tmy(3,3),Tmz(3,3);
//変化量
MatrixXd V_Speed(1, 3);
//変換後
MatrixXd V_Speed_T(1, 3);
// %EndTag(vars)%
void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof );
void chatterCallback(const pc_side_programs::Controller& controller_){
  if(START){
    V_Speed(0,1) = controller_.LS_Left_Right / 100.;
    if(controller_.Triangle)   V_Speed(0,0) = +(1./100.);
    else if(controller_.Cross) V_Speed(0,0) = -(1./100.);
    else V_Speed(0,0) = 0.;
    V_Speed(0,2) = controller_.LS_Up_Down / 100.;

    Rotation[2] += controller_.RS_Left_Right / 1.;
    Rotation[1] += -(controller_.RS_Up_Down / 1.);
    if(controller_.R1) Rotation[0] += 1./1.;
    else if(controller_.R2) Rotation[0] -= 1./1.;

    for(int i = 0; i < 3; i++)Rotation_rad[i] = (Rotation[i] / 180.) * PI;
    quaternion = tf::createQuaternionFromRPY(Rotation_rad[0],Rotation_rad[1],Rotation_rad[2]);

    Tmx << 1, 0, 0,
            0, cos(-Rotation_rad[0]), -sin(-Rotation_rad[0]),
            0, sin(-Rotation_rad[0]),  cos(-Rotation_rad[0]);
    Tmy << cos(-Rotation_rad[1]), 0, sin(-Rotation_rad[1]),
          0, 1, 0,
          -sin(-Rotation_rad[1]), 0, cos(-Rotation_rad[1]);
    Tmz << cos(-Rotation_rad[2]), -sin(-Rotation_rad[2]), 0,
            sin(-Rotation_rad[2]), cos(-Rotation_rad[2]), 0,
            0, 0, 1;

    V_Speed_T = V_Speed * Tmx * Tmy * Tmz;
    Position[0] += V_Speed_T(0,0);
    Position[1] += V_Speed_T(0,1);
    Position[2] += V_Speed_T(0,2);
  }  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_test_panda");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("Arduino", 1000, chatterCallback);
  ros::Rate loop_rate(10);
  ros::AsyncSpinner spinner(10);
  spinner.start();
  //spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "r5s_arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
 /* Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
*/
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  move_group.setPlanningTime(0.1);
  move_group.setNumPlanningAttempts(1);
  move_group.setMaxVelocityScalingFactor(1);
  move_group.setMaxAccelerationScalingFactor(1);

  geometry_msgs::Pose target_pose1;
  tf::TransformListener listener;
  
  tf::Vector3 position;
  position = tf::Vector3( Position[0], Position[1], Position[2]);
  
  Tmx << 1, 0, 0,
          0, cos(-Rotation_rad[0]), -sin(-Rotation_rad[0]),
          0, sin(-Rotation_rad[0]),  cos(-Rotation_rad[0]);
  Tmy << cos(-Rotation_rad[1]), 0, sin(-Rotation_rad[1]),
         0, 1, 0,
         -sin(-Rotation_rad[1]), 0, cos(-Rotation_rad[1]);
  Tmz << cos(-Rotation_rad[2]), -sin(-Rotation_rad[2]), 0,
          sin(-Rotation_rad[2]), cos(-Rotation_rad[2]), 0,
          0, 0, 1;
          

  //  cout << "Tmx:\n" << Tmx << endl;
  //  cout << "Tmy:\n" << Tmy << endl;
  //  cout << "Tmz:\n" << Tmz << endl;

  //  V_Speed << 1, 1, 0;
  //  cout << "V:\n" << V_Speed * Tmx * Tmy * Tmz << endl;
  START = true;;
  while(ros::ok()){
    // tf::StampedTransform transform;
    // try{
    //   ros::Time now = ros::Time::now();
    //   listener.waitForTransform("/world", "/body6_link",
    //                            now, ros::Duration(1.0));
    //   listener.lookupTransform("/world", "/body6_link",
    //                            now, transform);
    // }
    // catch (tf::TransformException &ex) {
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(2.0).sleep();
    //   continue;
    // }
    // t_pose.position.x = Position[0];
    // t_pose.position.y = Position[1];
    // t_pose.position.z = Position[2];
    // t_pose.orientation.x = quaternion[0];
    // t_pose.orientation.y = quaternion[1];
    // t_pose.orientation.z = quaternion[2];
    // t_pose.orientation.w = quaternion[3];

    target_pose1.position.x = Position[0];
    target_pose1.position.y = Position[1];
    target_pose1.position.z = Position[2];
    target_pose1.orientation.x = quaternion[0];
    target_pose1.orientation.y = quaternion[1];
    target_pose1.orientation.z = quaternion[2];
    target_pose1.orientation.w = quaternion[3];
    
    ROS_INFO("datax:[%f]", target_pose1.position.x);
    ROS_INFO("datay:[%f]", target_pose1.position.y);
    ROS_INFO("dataz:[%f]", target_pose1.position.z);
    ROS_INFO("Rotationx:[%f]", Rotation[0]);
    ROS_INFO("Rotationy:[%f]", Rotation[1]);
    ROS_INFO("Rotationz:[%f]", Rotation[2]);
    cout << "Tmx:\n" << Tmx << endl;
    cout << "Tmy:\n" << Tmy << endl;
    cout << "Tmz:\n" << Tmz << endl;

    cout << "V:\n" << V_Speed_T << endl;
    
    //ROS_INFO("%f",move_group.getPlanningTime());
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
/*
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
*/
    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    
  visual_tools.deleteAllMarkers();
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    //visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    // Moving to a pose goal
    // ^^^^^^^^^^^^^^^^^^^^^
    //
    // Moving to a pose goal is similar to the step above
    // except we now use the move() function. Note that
    // the pose goal we had set earlier is still active
    // and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is
    // a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.

    /* Uncomment below line when working with a real robot */
     move_group.move(); 
     if(!move_group.move()){
        ROS_WARN("Could not move to prepare pose");
     }
    ros::spinOnce();//spinはアイドルループかも?
    loop_rate.sleep();
  }
}

