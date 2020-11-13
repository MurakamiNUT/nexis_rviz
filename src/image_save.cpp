/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <stdio.h> 
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image;
    ros::Subscriber sub_key;

    public:
    ImageConverter():it_(n){
      sub_image = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
      sub_key = n.subscribe("key", 1000, &ImageConverter::chatterCallback_key, this);
      cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter(){
      cv::destroyWindow(OPENCV_WINDOW);
    }
    cv_bridge::CvImagePtr cv_ptr;
    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    }

    int32_t key; 
    void chatterCallback_key(const std_msgs::Int32& msg){
      key = msg.data;
      ROS_INFO("data:[%d]", key);
      const char *log;
      switch(key){
          case 97:
          break;
      }
    }
    
};


// void chatterCallback_image(const sensor_msgs::Image& msg){
//     ROS_INFO("image");
//     picture = msg;
// }
int main(int argc, char **argv){
    ros::init(argc, argv, "image_save");
    // ImageConverter ic;
    // while(ros::ok()){
    //     ros::spinOnce();
    // }
    ros::spin();
    return 0;
}