#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pc_side_programs/Controller.h>
#include "std_msgs/Float32MultiArray.h"

#include <string>
#include <math.h>

float position[3] = {};
void chatterCallback(const std_msgs::Float32MultiArray& msg){
    position[0] = 0;
    position[1] = msg.data[0];
    position[2] = 0;
}
int main(int argc, char **argv){
  ros::init(argc, argv, "joint_state_test");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("Servo_Angle", 1000, chatterCallback);
  //publisher
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    int count;
    sensor_msgs::JointState js0;
    js0.header.stamp = ros::Time::now();
    js0.name.resize(3);
    js0.name[0]="body1_joint";
    js0.name[1]="body2_joint";
    js0.name[2]="body3_joint";
    js0.position.resize(3);
    js0.position[0]= position[0];
    js0.position[1]= position[1];
    js0.position[2]= position[2];
    joint_pub.publish(js0);
    count++;

    ros::spinOnce();
    loop_rate.sleep();
  } 
 // return 0;
}