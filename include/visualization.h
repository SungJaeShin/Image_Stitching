#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/stitching.hpp"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

void drawimage(cv::Mat image1, std::vector<cv::KeyPoint> img1_kpt)
{
    // Draw image keypoints 
    cv::Mat outImg;
    cv::drawKeypoints(image1, img1_kpt, outImg);
    cv::imshow("Output", outImg);
}

void drawMasking(cv::Mat image1, cv::Mat image2, cv::Mat image3,
				 cv::Rect image1_rect, cv::Rect image2_rect, cv::Rect image3_rect)
{
	// [Apply Real Image CASE]
	if(MASKING_REAL)
	{
		cv::Mat copy_img1 = image1.clone();
		cv::Mat copy_img2 = image2.clone();
		cv::Mat copy_img3 = image3.clone();

		cv::rectangle(copy_img1, image1_rect, cv::Scalar(255, 0, 0), 2, 8, 0);
		cv::rectangle(copy_img2, image2_rect, cv::Scalar(255, 0, 0), 2, 8, 0);
		cv::rectangle(copy_img3, image3_rect, cv::Scalar(255, 0, 0), 2, 8, 0);

		cv::imshow("Masking 1", copy_img1);
		cv::imshow("Masking 2", copy_img2);
		cv::imshow("Masking 3", copy_img3);
		cv::waitKey(5);
	}

	// [Apply White Image CASE]
	if(!MASKING_REAL)
	{
		cv::Mat white_img1(image1.rows, image1.cols, CV_8UC1, cv::Scalar(255, 255, 255));
		cv::Mat white_img2(image2.rows, image2.cols, CV_8UC1, cv::Scalar(255, 255, 255));
		cv::Mat white_img3(image3.rows, image3.cols, CV_8UC1, cv::Scalar(255, 255, 255));

		cv::rectangle(white_img1, image1_rect, cv::Scalar(0, 0, 0), 1, 8, 0);
		cv::rectangle(white_img2, image2_rect, cv::Scalar(0, 0, 0), 1, 8, 0);
		cv::rectangle(white_img3, image3_rect, cv::Scalar(0, 0, 0), 1, 8, 0);

		cv::imshow("Masking 1", white_img1);
		cv::imshow("Masking 2", white_img2);
		cv::imshow("Masking 3", white_img3);
		cv::waitKey(5);
	}
}

void saveFeatureDescriptor(cv::Mat image1, cv::Mat image2, cv::Mat image3, int index, std_msgs::Header img1_time, std_msgs::Header img2_time, std_msgs::Header img3_time)
{
    if(SAVE_IMGS_WITH_FEATURES)
	{
		// Initialization
		cv::Mat img1_output, img2_output, img3_output;
		std::vector<cv::KeyPoint> img1_keypoint, img2_keypoint, img3_keypoint; 
		cv::Mat img1_des, img2_des, img3_des;

		// Detect ORB features
		cv::Ptr<cv::Feature2D> orb = cv::ORB::create();
		orb -> detectAndCompute(image1, cv::Mat(), img1_keypoint, img1_des);
		orb -> detectAndCompute(image2, cv::Mat(), img2_keypoint, img2_des);
		orb -> detectAndCompute(image3, cv::Mat(), img3_keypoint, img3_des);

		// Draw Feature matching 
		if(SAVE_IMGS_DESCRIPTOR_MATCHING)
		{
			cv::Ptr<cv::DescriptorMatcher> matcher_orb = cv::BFMatcher::create(cv::NORM_HAMMING);
			std::vector<cv::DMatch> img1_img2_matches;
			std::vector<cv::DMatch> img2_img3_matches;

			matcher_orb -> match(img1_des, img2_des, img1_img2_matches);
			matcher_orb -> match(img2_des, img3_des, img2_img3_matches);

			ROS_WARN("descriptor channel: %d", img1_des.channels());

			if(img1_des.cols != 0 && img2_des.cols != 0)
			{
				std::sort(img1_img2_matches.begin(), img1_img2_matches.end());
				std::sort(img2_img3_matches.begin(), img2_img3_matches.end());

				const int img1_img2_match_size = img1_img2_matches.size();
				const int img2_img3_match_size = img2_img3_matches.size();

				std::vector<cv::DMatch> img1_img2_good_matches(img1_img2_matches.begin(), img1_img2_matches.begin() + (int)(img1_img2_match_size * 0.5f));
				std::vector<cv::DMatch> img2_img3_good_matches(img2_img3_matches.begin(), img2_img3_matches.begin() + (int)(img2_img3_match_size * 0.5f));

				cv::Mat img1_img2_result, img2_img3_result;

				cv::drawMatches(image1, img1_keypoint, image2, img2_keypoint, img1_img2_good_matches, img1_img2_result, cv::Scalar::all(-1), cv::Scalar(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
				cv::drawMatches(image2, img2_keypoint, image3, img3_keypoint, img2_img3_good_matches, img2_img3_result, cv::Scalar::all(-1), cv::Scalar(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

				std::string save_match1_dir = "~/image_stitching/image_result/match_image/img1_img2_" + std::to_string(index) + "_.png";
				std::string save_match2_dir = "~/image_stitching/image_result/match_image/img2_img3_" + std::to_string(index) + "_.png";

				cv::imwrite(save_match1_dir, img1_img2_result);
				cv::imwrite(save_match2_dir, img2_img3_result);
			}
			else
				ROS_WARN("Not descriptor matching because of zero size !!");
		}

		// Detect FAST featuers
		// cv::FAST(image1, img1_keypoint, 20, true, cv::FastFeatureDetector::TYPE_9_16);
		// cv::FAST(image2, img2_keypoint, 20, true, cv::FastFeatureDetector::TYPE_9_16);
		// cv::FAST(image3, img3_keypoint, 20, true, cv::FastFeatureDetector::TYPE_9_16);

		// DrawKeypoints
		cv::drawKeypoints(image1, img1_keypoint, img1_output, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::drawKeypoints(image2, img2_keypoint, img2_output, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::drawKeypoints(image3, img3_keypoint, img3_output, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		
		// Just plot time two images to see closet time
		double time1 = img1_time.stamp.toSec();
		double time2 = img2_time.stamp.toSec();
		double time3 = img3_time.stamp.toSec();
		double img_width = image1.cols;
		double img_height = image1.rows;

		cv::Mat gap_img(img_height, 10, CV_8UC1, cv::Scalar(255, 255, 255));
		cv::Mat gap_img2(img_height, 10, CV_8UC1, cv::Scalar(255, 255, 255));
		cv::Mat mid_img;
		cv::Mat sync_orb_img;

		cv::cvtColor(gap_img, gap_img, cv::COLOR_GRAY2BGR);
		cv::cvtColor(gap_img2, gap_img2, cv::COLOR_GRAY2BGR);
		// Assume that two images (image1 and image2) have same height -> then it is okay to use hconcat function
		cv::hconcat(img1_output, gap_img, gap_img);
		cv::hconcat(gap_img, img2_output, mid_img);
		cv::hconcat(mid_img, gap_img2, gap_img2);
		cv::hconcat(gap_img2, img3_output, sync_orb_img);

		cv::Mat feature_notation(50, img_width + 10 + img_width + 10 + img_width, CV_8UC1, cv::Scalar(255, 255, 255));
		cv::cvtColor(feature_notation, feature_notation, cv::COLOR_GRAY2BGR);

		putText(feature_notation, "image1 time: " + std::to_string(time1), cv::Point2f(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
		putText(feature_notation, "image2 time: " + std::to_string(time2), cv::Point2f(20 + img_width + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
		putText(feature_notation, "image3 time: " + std::to_string(time3), cv::Point2f(20 + img_width + 10 + img_width + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);

		cv::vconcat(feature_notation, sync_orb_img, sync_orb_img);

		std::string save_orb_dir = "~/image_stitching/image_result/orb_image/orb_" + std::to_string(index) + "_.png";
		cv::imwrite(save_orb_dir, sync_orb_img);
	}
}

void saveSyncImgswithTime(cv::Mat image1, cv::Mat image2, cv::Mat image3, int index, std_msgs::Header img1_time, std_msgs::Header img2_time, std_msgs::Header img3_time)
{
    if(SAVE_SYNC_IMG_WITH_TIME)
	{
		// Just plot time two images to see closet time
		double time1 = img1_time.stamp.toSec();
		double time2 = img2_time.stamp.toSec();
		double time3 = img3_time.stamp.toSec();
		double img_width = image1.cols;
		double img_height = image1.rows;

		cv::Mat gap_img(img_height, 10, CV_8UC1, cv::Scalar(255, 255, 255));
		cv::Mat gap_img2(img_height, 10, CV_8UC1, cv::Scalar(255, 255, 255));
		cv::Mat mid_img;
		cv::Mat sync_img;

		cv::cvtColor(gap_img, gap_img, cv::COLOR_GRAY2BGR);
		cv::cvtColor(gap_img2, gap_img2, cv::COLOR_GRAY2BGR);
		// Assume that two images (image1 and image2) have same height -> then it is okay to use hconcat function
		cv::hconcat(image1, gap_img, gap_img);
		cv::hconcat(gap_img, image2, mid_img);
		cv::hconcat(mid_img, gap_img2, gap_img2);
		cv::hconcat(gap_img2, image3, sync_img); 

		cv::Mat notation(50, img_width + 10 + img_width + 10 + img_width, CV_8UC1, cv::Scalar(255, 255, 255));
		cv::cvtColor(notation, notation, cv::COLOR_GRAY2BGR);

		putText(notation, "image1 time: " + std::to_string(time1), cv::Point2f(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
		putText(notation, "image2 time: " + std::to_string(time2), cv::Point2f(20 + img_width + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);
		putText(notation, "image3 time: " + std::to_string(time3), cv::Point2f(20 + img_width + 10 + img_width + 10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 3);

		cv::vconcat(notation, sync_img, sync_img);

		std::string save_sync_dir = "~/image_stitching/image_result/sync_image/sync_" + std::to_string(index) + "_.png";
		cv::imwrite(save_sync_dir, sync_img);
	}
}