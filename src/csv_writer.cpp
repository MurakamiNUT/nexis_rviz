#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <string>
#include <fstream>
#include <std_msgs/Int32.h>
//実験時のデータ保存用プログラム
//topicに配信されているデータを持ってきてcsvに記述するだけです。
//記録開始時間保存用
ros::Time record_start;
int32_t key;
void chatterCallback_key(const std_msgs::Int32& key_data){
    key = key_data.data;
    ROS_INFO("data:[%d]", key);
    switch(key){
        case 105://i//記録開始
            sleep(1);
            record_start = ros::Time::now();
            ROS_INFO("::::Record_start::::");
        break;

        case 111://o//記録停止
            sleep(1);
        break;
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "csv_writer");
    ros::NodeHandle nh;
    ros::Subscriber sub_key = nh.subscribe("key", 1, chatterCallback_key);
    // ros::Subscriber arm_mode_sub = nh.subscribe("arm_mode", 1, chatterCallback_arm_mode);
    // ros::Subscriber mode_ik_sub  = nh.subscribe("mode_ik", 1, chatterCallback_mode_ik);
    // ros::Subscriber s_mode_sub   = nh.subscribe("s_mode", 1, chatterCallback_s_mode);
    // ros::Publisher arm_mode_text_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("arm_mode_text", 1);
    // ros::Publisher mode_ik_text_pub  = nh.advertise<jsk_rviz_plugins::OverlayText>("mode_ik_text", 1);
    // ros::Publisher s_mode_text_pub  = nh.advertise<jsk_rviz_plugins::OverlayText>("s_mode_text", 1);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {

      ROS_INFO("Time:: %f", ros::Time::now().toSec() - record_start.toSec());
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}