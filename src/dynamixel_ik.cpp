#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <beginner_tutorials/Controller.h>
#include "std_msgs/Float32MultiArray.h"

//#include <std/string>
#include <math.h>
#include <kdl_parser/kdl_parser.hpp>
//#include <kdl/tree.hpp>


//float position[3] = {};

void chatterCallback(const std_msgs::Float32MultiArray& msg){
}

int main(int argc, char **argv){
  ros::init(argc, argv, "dynamixel_ik");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("Servo_Angle", 1000, chatterCallback);
  //publisher
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

  ros::Rate loop_rate(10);
 
    KDL::Tree my_tree;
  // if (!kdl_parser::treeFromFile("./src/nexis_rviz/urdf/crane_plus.urdf", my_tree)){
  //    ROS_ERROR("Failed to construct kdl tree");
  //    return false;
  // }

  while (ros::ok()){

    ros::spinOnce();
    loop_rate.sleep();
  } 
 // return 0;
}