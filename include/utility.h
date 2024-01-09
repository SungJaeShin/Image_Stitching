#ifndef UTILITY
#define UTILITY

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/stitching.hpp"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

void cvType(cv::Mat des)
{
    /* #define CV_8U   0
	   #define CV_8S   1
	   #define CV_16U  2
       #define CV_16S  3
	   #define CV_32S  4
	   #define CV_32F  5
	   #define CV_64F  6
	   #define CV_USRTYPE1 7 */

    if(des.type() == CV_8U)
		ROS_WARN("des type is CV_8U"); 
	if(des.type() == CV_8S)
		ROS_WARN("des type is CV_8S");
	if(des.type() == CV_16U)
		ROS_WARN("des type is CV_16U"); 
	if(des.type() == CV_16S)
		ROS_WARN("des type is CV_16S"); 
	if(des.type() == CV_32S)
		ROS_WARN("des type is CV_32S"); 
	if(des.type() == CV_32F)
		ROS_WARN("des type is CV_32F"); 
	if(des.type() == CV_64F)
		ROS_WARN("des type is CV_64F"); 
}

cv::Mat sensorMsg2cvMat(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
		ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGRA8);
    }
    else
		ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGRA8);

    cv::Mat img = ptr->image.clone();
	cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

    return img;
}

sensor_msgs::Image cvMat2sensorMsg(cv::Mat image, std_msgs::Header header)
{
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img;

	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
	img_bridge.toImageMsg(img);

	return img;
}

std::vector<cv::KeyPoint> convert_Point2f_to_KeyPoint(std::vector<cv::Point2f> points_2d)
{
    std::vector<cv::KeyPoint> points_kpt;
    for(std::vector<cv::Point2f>::const_iterator it = points_2d.begin(); it != points_2d.end(); it++) 
    {
        cv::KeyPoint tmp(*it, 8);
        points_kpt.push_back(tmp);
    }
    return points_kpt;
}

// Code from VINS-Mono 
// Ref site: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
void reduceVector(std::vector<cv::Point2f> &points_2d, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(points_2d.size()); i++)
        if (status[i])
            points_2d[j++] = points_2d[i];
    points_2d.resize(j);
}

void reduceVector(std::vector<cv::KeyPoint> &points_kpt, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(points_kpt.size()); i++)
        if (status[i])
            points_kpt[j++] = points_kpt[i];
    points_kpt.resize(j);
}

void reduceVector(std::vector<cv::DMatch> &matches, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(matches.size()); i++)
        if (status[i])
            matches[j++] = matches[i];
    matches.resize(j);
}

#endif