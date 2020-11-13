#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <string>
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

jsk_rviz_plugins::OverlayText Arm_mode_text;
jsk_rviz_plugins::OverlayText Mode_ik_text;
jsk_rviz_plugins::OverlayText S_mode_text;
void chatterCallback_arm_mode(const std_msgs::Bool& msg){
    if(msg.data) Arm_mode_text.text = "Manipulation";
    else Arm_mode_text.text = "Crawler";
}
void chatterCallback_mode_ik(const std_msgs::Bool& msg){
    if(msg.data) Mode_ik_text.text = "Inverse";
    else Mode_ik_text.text = "Forward";
}
void chatterCallback_s_mode(const std_msgs::Int32& msg){
    if(msg.data == 0) S_mode_text.text = "slow";
    else if(msg.data == 1) S_mode_text.text = "normal";
    else if(msg.data == 2) S_mode_text.text = "fast";
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_overlay");
    ros::NodeHandle nh;
    ros::Subscriber arm_mode_sub = nh.subscribe("arm_mode", 1, chatterCallback_arm_mode);
    ros::Subscriber mode_ik_sub  = nh.subscribe("mode_ik", 1, chatterCallback_mode_ik);
    ros::Subscriber s_mode_sub   = nh.subscribe("s_mode", 1, chatterCallback_s_mode);
    ros::Publisher arm_mode_text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("arm_mode_text", 1);
    ros::Publisher mode_ik_text_pub  = nh.advertise<jsk_rviz_plugins::OverlayText>("mode_ik_text", 1);
    ros::Publisher s_mode_text_pub  = nh.advertise<jsk_rviz_plugins::OverlayText>("s_mode_text", 1);

    ros::Rate loop_rate(5);

    Arm_mode_text.line_width = 1;
    Arm_mode_text.text_size = 14;
    Arm_mode_text.font = "Ubuntu";
    Arm_mode_text.text = "hello";

    Mode_ik_text.line_width = 1;
    Mode_ik_text.text_size = 14;
    Mode_ik_text.font = "Ubuntu";
    Mode_ik_text.text = "hello";

    S_mode_text.line_width = 1;
    S_mode_text.text_size = 14;
    S_mode_text.font = "Ubuntu";
    S_mode_text.text = "hello";
    while (ros::ok())
    {

        arm_mode_text_pub.publish(Arm_mode_text);
        mode_ik_text_pub.publish(Mode_ik_text);
        s_mode_text_pub.publish(S_mode_text);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}