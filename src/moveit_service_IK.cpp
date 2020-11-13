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
#include <moveit_msgs/GetPositionFK.h>

#include <sensor_msgs/JointState.h>
#include <pc_side_programs/Controller.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
// void rotation_matrix_calc();

#define PI 3.1415926535
// using namespace std;
using Eigen::MatrixXd;
#define ALL_SPEED 30  //deg/sのつもり（制御周期早かったりするとおかしくなる）
#define CONTROL_TIME 10//制御周期
// double Position[3] = {0.12435, 0., 0.20339};//x y z
double Position[3] = {0., 0., 0.};//x y z
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
//過去関節角度
double joint_state_old[6] = {0,0,0,0,0,0};
//fk用
double Servo_V[6] = {};
float Arm_C[6] ={//毎秒どれくらい動くか
 ((3.14159265 / 180) * ALL_SPEED) / 1.5,
 ((3.14159265 / 180) * ALL_SPEED) / 1.5,
 ((3.14159265 / 180) * ALL_SPEED) / 1.5,
 (3.14159265 / 180) * ALL_SPEED,
 (3.14159265 / 180) * ALL_SPEED,
 (3.14159265 / 180) * ALL_SPEED
};
double Priset[7][6] = {
  {0.,0.,0.,0.,0.,0.},
  {0.,0.750270,-1.012291,2.663706,0.,0.},
  {0.,0.868296,-0.733038,1.435751,0.,0.},
  {0.,0.174580,-0.383972,0.032809,-3.143693,0.},
  {0.,1.035520,-1.745329,0.04125,0.},
  {0.,0.115883,-0.174533,0.041250,0.,0.},
  {0.,1.037497,-1.366045,0.305249,0.017536,0.}
};

//基礎速度、22,24で加速減速
float basic_speed = 1.;
float ss[3] = {0.5, 1., 1.5};
int s_mode = 1;
//コントローラ入力をもらったらtrue
// bool data_get = false;
//回転行列の計算
// void ik_pub();
// class service_IK;
// class base{
//   protected:
//     base(int argc, char** argv,const char* node_name){
//      ros::init(argc, argv, node_name);
//     }
// };
class service_IK{
  private:
  public:
    service_IK();
    // ~service_IK(int argc, char** argv);
    void chatterCallback(const pc_side_programs::Controller& controller_);
    void chatterCallback_key(const std_msgs::Int32& key_data);
    void ik_pub();
    void ik_data_process();
    void pose_set(int line);
    //可操作度計算用
    void manipulability_measure();
    //パラメータ更新よう
    void param_updata();
    // int main(int argc, char** argv);
    //コントローラ入力
    ros::Subscriber sub;
    //キーボード入力
    ros::Subscriber sub_key;
    // ros::Rate* loop_rate_;
    ros::Time ros_start;

    ros::Publisher robot_state_publisher;
    //可操作度配信
    ros::Publisher manipulability_measure_pub;
    //ARM_MODE配信
    ros::Publisher arm_mode_pub;
    //MODE_IK配信
    ros::Publisher mode_ik_pub;
    //速度配信
    ros::Publisher s_mode_pub;
    ros::NodeHandle node_handle;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::JointModelGroup* joint_model_group;
    robot_state::RobotStatePtr kinematic_state;
    //逆動に必要な宣言関係
    ros::ServiceClient service_client;
    moveit_msgs::GetPositionIK::Request service_request;
    moveit_msgs::GetPositionIK::Response service_response;
    //順動
    ros::ServiceClient service_client_fk;
    moveit_msgs::GetPositionFK::Request service_request_fk;
    moveit_msgs::GetPositionFK::Response service_response_fk;
    //rvizへの反映
    moveit_msgs::DisplayRobotState msg;

    //system start flag
    bool START = false;
    //アーム操作入力検出
    bool CHANGE = false;
    bool MODE_IK = true;//IKを使うか
    bool ARM_MODE = false;
    //配信用
    std_msgs::Bool Mode_ik;
    std_msgs::Bool Arm_mode;
    std_msgs::Int32 S_mode_step;
    //速度変更ボタンようフラグ
    bool button_flg[2] = {};
    //規定しせい用
    bool move_pose_flg = false;
    //可操作度
    Eigen::MatrixXd jacobian;
    std_msgs::Float32 Manipulability_Measure;
    // Eigen::Vector3d reference_point_position(0.0,0.0,0.0);

};

service_IK::service_IK(){
  sub = node_handle.subscribe("Arduino", 1000, &service_IK::chatterCallback, this);
  sub_key = node_handle.subscribe("key", 1000, &service_IK::chatterCallback_key, this);
  //ros::ServiceClient service_client;
  service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
  service_client_fk = node_handle.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
  robot_state_publisher = node_handle.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1000);
  manipulability_measure_pub = node_handle.advertise<std_msgs::Float32>("manipulability_measure", 1);
  arm_mode_pub = node_handle.advertise<std_msgs::Bool>("arm_mode",1);
  mode_ik_pub = node_handle.advertise<std_msgs::Bool>("mode_ik",1);
  s_mode_pub = node_handle.advertise<std_msgs::Int32>("s_mode",1);
  while (!service_client.exists()){
    ROS_INFO("Waiting for service");
    sleep(1.0);
  }
  quaternion[0] = 0.;
  quaternion[1] = 0.;
  quaternion[2] = 0.;
  quaternion[3] = 1.;
  service_request.ik_request.group_name = "r5s_arm";
  service_request.ik_request.pose_stamped.header.frame_id = "body6_link";
  service_request.ik_request.pose_stamped.pose.position.x = Position[0];
  service_request.ik_request.pose_stamped.pose.position.y = Position[1];
  service_request.ik_request.pose_stamped.pose.position.z = Position[2];

  service_request.ik_request.pose_stamped.pose.orientation.x = quaternion[0];
  service_request.ik_request.pose_stamped.pose.orientation.y = quaternion[1];
  service_request.ik_request.pose_stamped.pose.orientation.z = quaternion[2];
  service_request.ik_request.pose_stamped.pose.orientation.w = quaternion[3];

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //robot_model::RobotModelPtr kinematic_model;
  kinematic_model = robot_model_loader.getModel();
  // robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  //const robot_state::JointModelGroup* joint_model_group;
  joint_model_group = kinematic_model->getJointModelGroup("r5s_arm");

  /* Get the names of the joints in the r5s_arm*/
  service_request.ik_request.robot_state.joint_state.name = joint_model_group->getJointModelNames();
  for(int i = 0; i < 6; i++)  service_request.ik_request.robot_state.joint_state.name[i] = service_request.ik_request.robot_state.joint_state.name[i+2];
  service_request.ik_request.robot_state.joint_state.name.resize(6);

  /* Get the joint values and put them into the message, this is where you could put in your own set of values as
   * well.*/
  kinematic_state->setToDefaultValues();
  kinematic_state->copyJointGroupPositions(joint_model_group,
                                           service_request.ik_request.robot_state.joint_state.position);

  service_client.call(service_request, service_response);
  ROS_INFO_STREAM(
      "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);

/**********************************************/
//fk用の初期化処理
  service_request_fk.fk_link_names = joint_model_group->getLinkModelNames();
  for(int i = 0; i < 6; i++)  service_request_fk.fk_link_names[i] = service_request_fk.fk_link_names[i+2];
  service_request_fk.fk_link_names.resize(6);

  service_request_fk.robot_state.joint_state.name = joint_model_group->getJointModelNames();
  for(int i = 0; i < 6; i++)  service_request_fk.robot_state.joint_state.name[i] = service_request_fk.robot_state.joint_state.name[i+2];
  service_request_fk.robot_state.joint_state.name.resize(6);

  kinematic_state->copyJointGroupPositions(joint_model_group,
                                            service_request_fk.robot_state.joint_state.position);
  service_client_fk.call(service_request_fk, service_response_fk);
  ROS_INFO_STREAM(
      "Result: " << ((service_response_fk.error_code.val == service_response_fk.error_code.SUCCESS) ? "True " : "False ")
                 << service_response_fk.error_code.val);
  // ROS_INFO_STREAM(service_request);
  // ROS_INFO_STREAM(service_response);
/**********************************************/
  /* Visualize the result*/
  kinematic_state->setVariableValues(service_response.solution.joint_state);
  robot_state::robotStateToRobotStateMsg(*kinematic_state, msg.state);
  msg.state.joint_state.name.resize(6);
  msg.state.joint_state.name[0] = "body1_joint";
  msg.state.joint_state.name[1] = "body2_joint";
  msg.state.joint_state.name[2] = "body3_joint";
  msg.state.joint_state.name[3] = "body4_joint";
  msg.state.joint_state.name[4] = "body5_joint";
  msg.state.joint_state.name[5] = "body6_joint";
  msg.state.joint_state.position.resize(6);
  for(int i=0;i < 6;i++){
    msg.state.joint_state.position[i] = service_response.solution.joint_state.position[i];
    joint_state_old[i] = msg.state.joint_state.position[i];
    ROS_INFO("%f",service_response.solution.joint_state.position[i]);
  }
  // ROS_INFO_STREAM(*kinematic_state);
  // ROS_INFO_STREAM(msg.state);
  robot_state_publisher.publish(msg);
  param_updata();

  // Sleep to let the message go through 
  ros::Duration(2.0).sleep();
  START = true;
  
}
//キーボード入力
int32_t key;
void service_IK::chatterCallback_key(const std_msgs::Int32& key_data){
  key = key_data.data;
  ROS_INFO("data:[%d]", key);
  const char *log;
  switch(key){
    case 97://a//規定しせい
      if(!move_pose_flg){
        pose_set(5);
      }
    break;
    case 102://f//ふせ
      if(!move_pose_flg){
        pose_set(0);
      }
    break;
    case 104://h//高い
      if(!move_pose_flg){
        pose_set(4);
      }
    break;
    case 109://m//中くらい
      if(!move_pose_flg){
        pose_set(6);
      }
    break;
    case 115://s//下
      if(!move_pose_flg){
        pose_set(2);
      }
    break;
    case 98://b//後ろ
      if(!move_pose_flg){
        pose_set(3);
      }
    break;
    case 114://r//ロボット
      if(!move_pose_flg){
        pose_set(1);
      }
    break;
    case 99://c
    for(int i=0; i<6; i++){
      if(i == 5) ROS_INFO("joint[%d]:: %f",i,fmod(msg.state.joint_state.position[i],6.282));
      else  ROS_INFO("joint[%d]:: %f",i,msg.state.joint_state.position[i]);
    }
    break;
  }
}
//コントローラ入力
void service_IK::chatterCallback(const pc_side_programs::Controller& controller_){

  if(controller_.R3) ARM_MODE = true;
  if(controller_.L3) ARM_MODE = false;
  if(ARM_MODE){
    if(controller_.Right_23){
      ik_data_process();
      MODE_IK = true;
    }
    if(controller_.Left_20) MODE_IK = false;
    if(controller_.Right_22){
      if(!button_flg[0]){
        //  basic_speed += 0.2;
        s_mode++;
        if(s_mode >= 2) s_mode = 2;
        basic_speed = ss[s_mode];
        button_flg[0] = true;
      }
      ROS_INFO("%f",basic_speed);
    }
    else button_flg[0] = false;

    if(controller_.Right_24){
      if(!button_flg[1]){
        // if(!(basic_speed <= 0.4)) basic_speed -= 0.2;
        s_mode--;
        if(s_mode <= 0) s_mode = 0;
        basic_speed = ss[s_mode];
        button_flg[1] = true;
      }
      ROS_INFO("%f",basic_speed);
    }
    else button_flg[1] = false;
    if(START){
      if(MODE_IK){
        V_Speed(0,1) = controller_.LS_Left_Right / 10. / CONTROL_TIME * basic_speed;
        if(controller_.Triangle)   V_Speed(0,0) = +(1./10.) / CONTROL_TIME * basic_speed;
        else if(controller_.Cross) V_Speed(0,0) = -(1./10.) / CONTROL_TIME * basic_speed;
        else V_Speed(0,0) = 0.;
        V_Speed(0,2) = controller_.LS_Up_Down / 10. / CONTROL_TIME * basic_speed;

        Rotation[2] += controller_.RS_Left_Right / 0.05 / CONTROL_TIME * basic_speed;
        Rotation[1] += -(controller_.RS_Up_Down / 0.05) / CONTROL_TIME * basic_speed;
        if(controller_.Circle) Rotation[0] += 1./0.02 / CONTROL_TIME * basic_speed;
        else if(controller_.Square) Rotation[0] -= 1./0.02 / CONTROL_TIME * basic_speed;

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

        if((controller_.LS_Left_Right  == 0.) &
          (controller_.Triangle        == 0)  &
          (controller_.Cross           == 0)  &
          (controller_.LS_Up_Down      == 0.) &
          (controller_.RS_Left_Right   == 0.) &
          (controller_.RS_Up_Down      == 0.) &
          (controller_.Circle              == 0)  &
          (controller_.Square              == 0)  ){
        CHANGE = false;
        }
        else  CHANGE = true;
        // ROS_INFO("%d",CHANGE);
      }
      else{
      //コントローラ値の受け取り
        Servo_V[0] = controller_.LS_Left_Right * (Arm_C[0] / CONTROL_TIME);
        Servo_V[1] = controller_.LS_Up_Down    * (Arm_C[1] / CONTROL_TIME);
        Servo_V[4] = controller_.RS_Left_Right * (Arm_C[4] / CONTROL_TIME);
        Servo_V[3] = controller_.RS_Up_Down    * (-Arm_C[3] / CONTROL_TIME);
        // Servo_V[7] = controller_.RS_Up_Down       * Arm_C[7];
        // if(controller_.L1)         Servo_V[6] = Arm_C[6];
        // else if(controller_.L2)    Servo_V[6] = -Arm_C[6];
        // else Servo_V[6] = 0;

        if(controller_.Circle)        Servo_V[5] = (Arm_C[5] / CONTROL_TIME);
        else if(controller_.Square)   Servo_V[5] = (-Arm_C[5] / CONTROL_TIME);
        else Servo_V[5] = 0;

        if(controller_.Triangle)   Servo_V[2] = (-Arm_C[2] / CONTROL_TIME);
        else if(controller_.Cross) Servo_V[2] = (Arm_C[2] / CONTROL_TIME);
        else Servo_V[2] = 0;

        // if(controller_.Circle)   Servo_V[5] = Arm_C[5];
        // else if(controller_.Square) Servo_V[5] = -Arm_C[5];
        // else Servo_V[5] = 0;
    }  
    }
  }
  // data_get = true;
}

void service_IK::ik_pub(){
  //ik
  if(MODE_IK){
    if(CHANGE){
      //先端位置及び回転を入力
      int c_value = 1;
      service_request.ik_request.pose_stamped.pose.position.x = Position[0] / c_value;
      service_request.ik_request.pose_stamped.pose.position.y = Position[1] / c_value;
      service_request.ik_request.pose_stamped.pose.position.z = Position[2] / c_value;
      service_request.ik_request.pose_stamped.pose.orientation.x = quaternion[0] / c_value;
      service_request.ik_request.pose_stamped.pose.orientation.y = quaternion[1] / c_value;
      service_request.ik_request.pose_stamped.pose.orientation.z = quaternion[2] / c_value;
      service_request.ik_request.pose_stamped.pose.orientation.w = quaternion[3] / c_value;
      kinematic_state->copyJointGroupPositions(joint_model_group,
                                              service_request.ik_request.robot_state.joint_state.position);
      //接触判定を利用するか
      // service_request.ik_request.avoid_collisions = true;
      //IKサービスを利用
      service_client.call(service_request, service_response);
      ROS_INFO_STREAM(
          "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                      << service_response.error_code.val);
      //ロボットモデルにjointstateを渡してる
      kinematic_state->setVariableValues(service_response.solution.joint_state);
      //変化量確認用
      bool ACcheck[6] = {};
      float Th = 30.;//しきい値、[度]
      int c_math = 0;
      for(int i=0;i < 6;i++){
        if(fabs(fabs(joint_state_old[i]) - fabs(service_response.solution.joint_state.position[i])) < (Th * PI / 180.)){
          ACcheck[i] = true;
          c_math++;
        }
        else ACcheck[i] = false;
      }
      if(c_math >= 5){
        for(int i=0;i < 6;i++){
          if(i == 5){
            msg.state.joint_state.position[i] = fmod(service_response.solution.joint_state.position[i], 6.282);
            joint_state_old[i] = fmod(service_response.solution.joint_state.position[i],6.282);
          }
          else{
            msg.state.joint_state.position[i] = service_response.solution.joint_state.position[i];
            joint_state_old[i] = service_response.solution.joint_state.position[i];
          }
          // ROS_INFO("a[%d]: %f",i,fabs(fabs(joint_state_old[i]) - fabs(service_response.solution.joint_state.position[i])));
          // ROS_INFO("th[%d]: %f",i,(Th * PI / 180.));

          // ROS_INFO("joint_state_old[%d]: %f",i,(joint_state_old[i]));
          // ROS_INFO("jointNo[%d]: %f",i,(service_response.solution.joint_state.position[i]));
        }
      }
      // for(int i=0;i < 5;i++)  ROS_INFO("jointCheck[%d]: %d",i,ACcheck[i]);
      // else{
      //   for(int i=0;i < 6;i++){
      //     joint_state_old[i] = service_response.solution.joint_state.position[i];
      //   }
      // }
      // msg.state.joint_state.position[5] = service_response.solution.joint_state.position[5];
      // joint_state_old[5] = msg.state.joint_state.position[5];
      // ROS_INFO("c_math: %d",c_math);
      c_math = 0;
      // ROS_INFO_STREAM(msg.state);

      // ROS_INFO("datax:[%f]", Position[0]);
      // ROS_INFO("datay:[%f]", Position[1]);
      // ROS_INFO("dataz:[%f]", Position[2]);
      // ROS_INFO("Rotationx:[%f]", Rotation[0]);
      // ROS_INFO("Rotationy:[%f]", Rotation[1]);
      // ROS_INFO("Rotationz:[%f]", Rotation[2]);
      // ROS_INFO("quaternion:x:[%f]", quaternion[0]);
      // ROS_INFO("quaternion:y:[%f]", quaternion[1]);
      // ROS_INFO("quaternion:z:[%f]", quaternion[2]);
      // ROS_INFO("quaternion:w:[%f]", quaternion[3]);

      robot_state_publisher.publish(msg);
    }
  }
  //fk
  else{
    for(int i=0;i < 6;i++){
      msg.state.joint_state.position[i] += Servo_V[i] * basic_speed;
      joint_state_old[i] = msg.state.joint_state.position[i];
    }
    kinematic_state->setVariableValues(msg.state.joint_state);
    robot_state_publisher.publish(msg);
  }
}

void service_IK::ik_data_process(){
  for(int i=0;i < 6;i++){
    joint_state_old[i] = msg.state.joint_state.position[i];
    service_request_fk.robot_state.joint_state.position[i] = msg.state.joint_state.position[i];
  }
  service_client_fk.call(service_request_fk, service_response_fk);
  Position[0] = service_response_fk.pose_stamped[5].pose.position.x - 0.12465;
  Position[1] = service_response_fk.pose_stamped[5].pose.position.y;
  Position[2] = service_response_fk.pose_stamped[5].pose.position.z - 0.20339;
  // Rotation_rad[0] = service_response_fk.pose_stamped[5].pose.orientation[0];
  // Rotation_rad[1] = service_response_fk.pose_stamped[5].pose.orientation[1];
  // Rotation_rad[2] = service_response_fk.pose_stamped[5].pose.orientation[2];
  quaternion[0] = service_response_fk.pose_stamped[5].pose.orientation.x;
  quaternion[1] = service_response_fk.pose_stamped[5].pose.orientation.y;
  quaternion[2] = service_response_fk.pose_stamped[5].pose.orientation.z;
  quaternion[3] = service_response_fk.pose_stamped[5].pose.orientation.w;
  tf::Matrix3x3(quaternion).getRPY(Rotation_rad[0],Rotation_rad[1],Rotation_rad[2]);
  for(int i = 0; i < 3; i++)Rotation[i] = Rotation_rad[i] / PI * 180.;
  // quaternion = tf::createQuaternionFromRPY(Rotation_rad[0],Rotation_rad[1],Rotation_rad[2]);
  for(int i = 0; i < 3; i++) ROS_INFO("POSITION::    %f", Position[i]);
  for(int i = 0; i < 4; i++) ROS_INFO("quaternion::  %f", quaternion[i]);
  for(int i = 0; i < 3; i++) ROS_INFO("rotation_rad::%f", Rotation_rad[i]);
  for(int i = 0; i < 3; i++) ROS_INFO("rotation::    %f", Rotation[i]);

  kinematic_state->setVariableValues(msg.state.joint_state);  
}

void service_IK::pose_set(int line){
  ARM_MODE = false;
  START = false;
  move_pose_flg = true;
  double transition_time = 5.;//[s]//遷移時間
  //各関節変化量
  double change_value[6] = {};
  for(int i=0; i<6; i++)  change_value[i] = Priset[line][i] - msg.state.joint_state.position[i];
  change_value[5] = Priset[line][5] - fmod(msg.state.joint_state.position[5],6.282);
  //1処理あたりの変化量
  double step_size[6] = {};
  for(int i=0; i<6; i++)  step_size[i] = change_value[i] / (CONTROL_TIME * transition_time);
  ros::Rate loop_rate(CONTROL_TIME);
  ros_start = ros::Time::now();
  while(transition_time > (ros::Time::now().toSec() - ros_start.toSec())){
    for(int i=0; i<6; i++)  msg.state.joint_state.position[i] += step_size[i];
    ros::spinOnce();
    loop_rate.sleep();
    robot_state_publisher.publish(msg);
  }
  for(int i=0; i<6; i++)  msg.state.joint_state.position[i]  = Priset[line][i];
  robot_state_publisher.publish(msg);
  ik_data_process();
  ARM_MODE = true;
  START = true;
  move_pose_flg = false;
}

//可操作度計算用
void service_IK::manipulability_measure(){
  Eigen::Vector3d reference_point_position(0., 0.,0.);
  //ヤコビ行列の取得
  kinematic_state->getJacobian(joint_model_group,
                                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                reference_point_position, jacobian);
  // ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  //正則行列なので省略形での計算//論文参照
  Manipulability_Measure.data = fabs(jacobian.determinant());
  // ROS_INFO_STREAM("manipulability_measure: \n" << fabs(jacobian.determinant()) << "\n");
  //結果同じだった
  // ROS_INFO_STREAM("manipulability_measure2: \n" << sqrt((jacobian * jacobian.transpose()).determinant()) << "\n");
  manipulability_measure_pub.publish(Manipulability_Measure);
}

void service_IK::param_updata(){
  Arm_mode.data = ARM_MODE;
  Mode_ik.data = MODE_IK;
  S_mode_step.data = s_mode;
  arm_mode_pub.publish(Arm_mode);
  mode_ik_pub.publish(Mode_ik);
  s_mode_pub.publish(S_mode_step);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_service_IK");
  service_IK service_ik;
  sleep(1.0);
  ros::AsyncSpinner spinner(CONTROL_TIME);
  spinner.start();
  ros::Rate loop_rate(CONTROL_TIME);

  while(ros::ok()){
    if(service_ik.START)  service_ik.ik_pub();
    service_ik.manipulability_measure();
    service_ik.param_updata();
    ros::spinOnce();//spinはアイドルループかも?
    loop_rate.sleep();
  }
  return 0;
}
