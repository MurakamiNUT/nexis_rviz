#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <string>
#include <fstream>
//実験時のデータ保存用プログラム
//topicに配信されているデータを持ってきてcsvに記述するだけです。
int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_writer");
    ros::NodeHandle nh;
    // ros::Subscriber arm_mode_sub = nh.subscribe("arm_mode", 1, chatterCallback_arm_mode);
    // ros::Subscriber mode_ik_sub  = nh.subscribe("mode_ik", 1, chatterCallback_mode_ik);
    // ros::Subscriber s_mode_sub   = nh.subscribe("s_mode", 1, chatterCallback_s_mode);
    // ros::Publisher arm_mode_text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("arm_mode_text", 1);
    // ros::Publisher mode_ik_text_pub  = nh.advertise<jsk_rviz_plugins::OverlayText>("mode_ik_text", 1);
    // ros::Publisher s_mode_text_pub  = nh.advertise<jsk_rviz_plugins::OverlayText>("s_mode_text", 1);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}