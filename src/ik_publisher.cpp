
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include <cmath>
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "pc_side_programs/Controller.h"
#include <std_msgs/Float32MultiArray.h>
//pc_side_programs::Controller controller_;
std_msgs::Float32MultiArray msg;
#define move_val 0.01
void chatterCallback(const pc_side_programs::Controller& controller_)
{

    msg.data[1] += controller_.LS_Left_Right * move_val;
    msg.data[2] += controller_.LS_Up_Down    * move_val;
    //msg.data[2] += controller_.RS_Up_Down    * 0.01;
    
    if(controller_.Triangle)      msg.data[0] += move_val;
    else if(controller_.Cross)    msg.data[0] -= move_val;
    if(1.030 < sqrt((msg.data[1] * msg.data[1]) + (msg.data[2] * msg.data[2]) + (msg.data[0] * msg.data[0]) )){
      for(int i = 0; i < 3; i++){
        if(msg.data[i] > 0.0) msg.data[i] -= move_val;
        else                  msg.data[i] += move_val;
      }
    }
    msg.data[3] = 0.0;
    msg.data[4] = 0.0;
    msg.data[5] = 0.0;
    msg.data[6] = 0.0;
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ik_publisher");
  ros::NodeHandle n;
  msg.data.resize(7);
  
  msg.data[0] = 0.0572315;
  msg.data[1] = 0;
  msg.data[2] = 0.18326;
  
  ros::Subscriber sub = n.subscribe("Arduino", 1000, chatterCallback);
  ros::Publisher tip_data_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 1000);
  ros::Rate loop_rate(4);

while(ros::ok()){
  tip_data_pub.publish(msg);
  ros::spinOnce();//spinはアイドルループかも?
  loop_rate.sleep();
}
}
// %EndTag(FULLTEXT)%
