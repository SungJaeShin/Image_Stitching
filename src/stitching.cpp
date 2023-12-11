#include "../include/paramsetting.h"
#include "../include/extractor.h"
#include "../include/descriptor.h"
#include "../include/error_filter.h"
#include "../include/opencv_setting.h"
#include "../include/stitch.h"
#include "../include/preprocessing.h"
#include "../include/outlier_rejection.h"

std::queue<sensor_msgs::ImageConstPtr> image1_buf;
std::queue<sensor_msgs::ImageConstPtr> image2_buf;
std::queue<sensor_msgs::ImageConstPtr> image3_buf;

std::mutex m_buf;
ros::Publisher pub_pano_img;

int sequence = 0;

void makePanoramaImage(cv::Mat image1, cv::Mat image2, cv::Mat image3, cv::Mat &pano, int index, std_msgs::Header img1_time, std_msgs::Header img2_time, std_msgs::Header img3_time)
{
	// Check image read correctly
	if(image1.cols == 0 || image2.cols == 0 || image3.cols == 0)
	{
		ROS_WARN("Empty Image1 or Image2 or Image3");
		return;
	}

	#ifdef HAVE_OPENCV_XFEATURES2D
    	ROS_WARN("USING_SURF_ALGORITHM"); // When using xfeature2d, then feature extraction algorithm is SURF
	#endif
    	ROS_WARN("USING_ORB_ALGORITHM"); // When not use xfeature2d, then feature extraction algorithm is ORB

	// Image Preprocessing (Histogram Equalization)
	HistogramEqualization(image1, image2, image3, img1_time, img2_time, img3_time);

	// Check OpenCV Error
	if(!checkOpenCVerror(image1, image2, image3))
		return;

	// Set Camera Parameters
	std::vector<cv::detail::CameraParams> cams;
	initCamParam(cams);

	// Set Image Masking 
	std::vector<std::vector<cv::Rect>> masks;
	std::vector<std::vector<cv::Rect>> masks_multi;
	initMasking(image1, image2, image3, masks, masks_multi);

	// Panorama Stitching
	std::vector<cv::Mat> imgs;
	imgs.push_back(image1);
	imgs.push_back(image2);
	imgs.push_back(image3);
	cv::Mat panorama;

	if(!opencv_based_stitching(imgs, masks_multi, cams, panorama))
		return;

	// Panorama Outlier Rejection !! 
	double panorama_width = panorama.cols;
	double panorama_height = panorama.rows;
	if(panorama_width < 700)
		return;
	if(panorama_width > 1500)
		return;
	if(panorama_height < 350)
		return;
	if(panorama_height > 500)
		return;

	// Panorama Image Resize !!
	if(RESIZE)
	{
		double height = image1.cols / 4;
		double width = image1.rows / 4;
		cv::resize(panorama, panorama, cv::Size(height, width), CV_INTER_LINEAR);
	}

	// Visualization and Save Panorama Results
	cv::Mat pano_notation(50, panorama_width, CV_8UC1, cv::Scalar(255, 255, 255));
	cv::cvtColor(pano_notation, pano_notation, cv::COLOR_GRAY2BGR);
	putText(pano_notation, "Pano Width: " + std::to_string(panorama_width) + ", Pano Height: " + std::to_string(panorama_height), cv::Point2f(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
	cv::vconcat(pano_notation, panorama, panorama);

	std::string save_img1_dir = "~/image_stitching/image_result/raw_image/img1_" + std::to_string(index) + "_.png";
	std::string save_img2_dir = "~/image_stitching/image_result/raw_image/img2_" + std::to_string(index) + "_.png";
	std::string save_img3_dir = "~/image_stitching/image_result/raw_image/img3_" + std::to_string(index) + "_.png";
	std::string save_pano_dir = "~/image_stitching/image_result/panorama_image/pano_" + std::to_string(index) + "_.png";

	cv::imwrite(save_img1_dir, image1);
	cv::imwrite(save_img2_dir, image2);
	cv::imwrite(save_img3_dir, image3);
	cv::imwrite(save_pano_dir, panorama);

	printf("\033[1;32m Successfully make panorama image !! \033[0m \n");
	std::cout << "Pano width: " << panorama.cols << ", Pano Height: " << panorama.rows << std::endl;
	pano = panorama;
}

void img1_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
	m_buf.lock();
	image1_buf.push(image_msg);
	m_buf.unlock();
}

void img2_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
	m_buf.lock();
	image2_buf.push(image_msg);
	m_buf.unlock();
}

void img3_callback(const sensor_msgs::ImageConstPtr &image_msg)
{
	m_buf.lock();
	image3_buf.push(image_msg);
	m_buf.unlock();
}

void sync_process()
{
	while(1)
	{
		sequence++;

		cv::Mat image1, image2, image3, pano_image;
		sensor_msgs::Image pano_msg;
		std_msgs::Header header;

		// Just plot time twp images to see closet time 
		std_msgs::Header cur_time_img1, cur_time_img2, cur_time_img3;

		m_buf.lock();
		if(!image1_buf.empty() && !image2_buf.empty() && !image3_buf.empty())
		{
			double time1 = image1_buf.front() -> header.stamp.toSec();
			double time2 = image2_buf.front() -> header.stamp.toSec();
			double time3 = image3_buf.front() -> header.stamp.toSec();

			if(time1 < time2 - 0.003 && time2 < time3 - 0.003)
			{
				image1_buf.pop();
				image2_buf.pop();
				printf("[Case1] throw img1 and throw img2\n");
			}
			else if(time1 > time2 + 0.003 && time3 > time1 + 0.003)
			{
				image2_buf.pop();
				image1_buf.pop();
				printf("[Case2] throw img2 and throw img1\n");
			}
			else if(time2 < time3 - 0.003 && time3 < time1 - 0.003)
			{
				image2_buf.pop();
				image3_buf.pop();
				printf("[Case3] throw img2 and throw img3\n");
			}
			else if(time2 > time3 + 0.003 && time1 > time2 + 0.003)
			{
				image3_buf.pop();
				image2_buf.pop();
				printf("[Case4] throw img3 and throw img2\n");
			}
			else if(time3 < time1 - 0.003 && time1 < time2 - 0.003)
			{
				image3_buf.pop();
				image1_buf.pop();
				printf("[Case5] throw img3 and throw img1\n");
			}
			else if(time3 > time1 + 0.003 && time2 > time3 + 0.003)
			{
				image1_buf.pop();
				image3_buf.pop();
				printf("[Case6] throw img1 and throw img3\n");
			}
			else
			{
				// Just plot time two images to see closet time
				cur_time_img1 = image1_buf.front() -> header;
				cur_time_img2 = image2_buf.front() -> header;
				cur_time_img3 = image3_buf.front() -> header;

				header = image1_buf.front()->header;
				image1 = sensorMsg2cvMat(image1_buf.front());
				image1_buf.pop();
				image2 = sensorMsg2cvMat(image2_buf.front());
				image2_buf.pop();
				image3 = sensorMsg2cvMat(image3_buf.front());
				image3_buf.pop();
				//printf("find img0 and img1\n");

				// Get Time Consumption ! 
				auto start = std::chrono::high_resolution_clock::now();
				makePanoramaImage(image1, image2, image3, pano_image, sequence, cur_time_img1, cur_time_img2, cur_time_img3);
				auto finish = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
				std::cout << "Time duration: " << (double)duration.count() / 1000000 << " sec" << std::endl;
			}
		}
		m_buf.unlock();

		// Add cur_time_img1, cur_time_img2 to see closet time
		pano_msg = cvMat2sensorMsg(pano_image, header);
		pub_pano_img.publish(pano_msg);

		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stitching_node");
	ros::NodeHandle nh;

    ros::Subscriber sub_img1 = nh.subscribe("/left/camera/infra1/image_rect_raw", 100, img1_callback);
    ros::Subscriber sub_img2 = nh.subscribe("/front/camera/infra1/image_rect_raw", 100, img2_callback);
	ros::Subscriber sub_img3 = nh.subscribe("/right/camera/infra1/image_rect_raw", 100, img3_callback);
    
	pub_pano_img = nh.advertise<sensor_msgs::Image>("Panorama_Image", 1000);

	if(HIST_EQUA)
	{
		pub_img1_equalization = nh.advertise<sensor_msgs::Image>("Equalization_image1", 1000);
		pub_img2_equalization = nh.advertise<sensor_msgs::Image>("Equalization_image2", 1000);
		pub_img3_equalization = nh.advertise<sensor_msgs::Image>("Equalization_image3", 1000);
	}

    std::thread sync_thread{sync_process};
    ros::spin();

	return 0;
}
