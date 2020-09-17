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
// using namespace std;
using Eigen::MatrixXd;
// using namespace visualization_msgs;
bool START = false;
double Position[3] = {0.12435, 0., 0.20339};//x y z
//オイラー
double Rotation[3] = {0, 0, 0};//roll pitch yaw
//ラジアン
double Rotation_rad[3] = {0, 0, 0};
tf::Quaternion quaternion;
// geometry_msgs::Pose t_pose;
// tf::Transform transform;
//行列作成
//回転行列
MatrixXd Tmx(3,3),Tmy(3,3),Tmz(3,3);
//変化量
MatrixXd V_Speed(1, 3);
//変換後
MatrixXd V_Speed_T(1, 3);
// %EndTag(vars)%
// void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof );
void chatterCallback(const pc_side_programs::Controller& controller_){
  if(START){
    V_Speed(0,1) = controller_.LS_Left_Right / 100.;
    if(controller_.Triangle)   V_Speed(0,0) = +(1./100.);
    else if(controller_.Cross) V_Speed(0,0) = -(1./100.);
    else V_Speed(0,0) = 0.;
    V_Speed(0,2) = controller_.LS_Up_Down / 100.;

    Rotation[2] += controller_.RS_Left_Right / 100.;
    Rotation[1] += -(controller_.RS_Up_Down / 100.);
    if(controller_.R1) Rotation[0] += 1./100.;
    else if(controller_.R2) Rotation[0] -= 1./100.;

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
  
  Tmx << 1, 0, 0,
          0, cos(-Rotation_rad[0]), -sin(-Rotation_rad[0]),
          0, sin(-Rotation_rad[0]),  cos(-Rotation_rad[0]);
  Tmy << cos(-Rotation_rad[1]), 0, sin(-Rotation_rad[1]),
         0, 1, 0,
         -sin(-Rotation_rad[1]), 0, cos(-Rotation_rad[1]);
  Tmz << cos(-Rotation_rad[2]), -sin(-Rotation_rad[2]), 0,
          sin(-Rotation_rad[2]), cos(-Rotation_rad[2]), 0,
          0, 0, 1;
          

  START = true;;
  int c_value = 10;
  while(ros::ok()){
    ROS_INFO("datax:[%f]", Position[0] / c_value);
    ROS_INFO("datay:[%f]", Position[1] / c_value);
    ROS_INFO("dataz:[%f]", Position[2] / c_value);
    ROS_INFO("Rotationx:[%f]", Rotation[0] / c_value);
    ROS_INFO("Rotationy:[%f]", Rotation[1] / c_value);
    ROS_INFO("Rotationz:[%f]", Rotation[2] / c_value);
    ROS_INFO("quaternion:x:[%f]", quaternion[0] / c_value);
    ROS_INFO("quaternion:y:[%f]", quaternion[1] / c_value);
    ROS_INFO("quaternion:z:[%f]", quaternion[2] / c_value);
    ROS_INFO("quaternion:w:[%f]", quaternion[3] / c_value);
    ros::spinOnce();//spinはアイドルループかも?
    loop_rate.sleep();
  }
}

