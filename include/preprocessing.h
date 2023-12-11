#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include <ros/ros.h>
#include "paramsetting.h"
#include "utility.h"

// Publish Histogram Eqaulization image
ros::Publisher pub_img1_equalization;
ros::Publisher pub_img2_equalization;
ros::Publisher pub_img3_equalization;

void HistogramEqualization(cv::Mat &image1, cv::Mat &image2, cv::Mat &image3, 
                           std_msgs::Header img1_time, std_msgs::Header img2_time, std_msgs::Header img3_time)
{
    // Image Pre Processing advised KwangYik Jung
	// https://gaussian37.github.io/vision-concept-histogram_equalization/  
	if(HIST_EQUA)
	{
		cv::cvtColor(image1, image1, cv::COLOR_BGRA2GRAY);
		cv::cvtColor(image2, image2, cv::COLOR_BGRA2GRAY);
		cv::cvtColor(image3, image3, cv::COLOR_BGRA2GRAY);

        if(!USE_CHALE)
		{
			cv::equalizeHist(image1, image1);
			cv::equalizeHist(image2, image2);
			cv::equalizeHist(image3, image3);
		}
		else
		{
			cv::Ptr<cv::CLAHE> chale = cv::createCLAHE(40.0, cv::Size(8, 8));
			chale -> apply(image1, image1);
			chale -> apply(image2, image2);
			chale -> apply(image3, image3);
		}

		cv::cvtColor(image1, image1, cv::COLOR_GRAY2BGR);
		cv::cvtColor(image2, image2, cv::COLOR_GRAY2BGR);
		cv::cvtColor(image3, image3, cv::COLOR_GRAY2BGR);

		sensor_msgs::Image equal_img1 = cvMat2sensorMsg(image1, img1_time);
		sensor_msgs::Image equal_img2 = cvMat2sensorMsg(image2, img2_time);
		sensor_msgs::Image equal_img3 = cvMat2sensorMsg(image3, img3_time);

		pub_img1_equalization.publish(equal_img1);
		pub_img2_equalization.publish(equal_img2); 
		pub_img3_equalization.publish(equal_img3); 
	}
}