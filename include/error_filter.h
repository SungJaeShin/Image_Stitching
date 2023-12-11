#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/stitching.hpp"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

bool checkOpenCVerror(cv::Mat image1, cv::Mat image2, cv::Mat image3)
{
    cv::Rect image1_rect(int(image1.cols/2), 0, int(image1.cols/2) , image1.rows);
	cv::Rect image2_rect(0, 0, image2.cols, image2.rows);
    cv::Rect image2_rect1(0, 0, int(image2.cols/2), image2.rows);
	cv::Rect image2_rect2(int(image2.cols/2), 0, int(image2.cols/2), image2.rows);
	cv::Rect image3_rect(0, 0, int(image3.cols/2), image3.rows);

    // -------------------------------------------------------------------------
	/* [CASE 1] Feature Extraction and Feature Matching using ORB Method !!!
				ORB Feature Extration + Bruth Force Matching using HAMMING */
	#if METHOD == 0
		cv::Mat des1, des2, des3;
		std::vector<cv::KeyPoint> kpt1, kpt2, kpt3;

		cv::Ptr<cv::Feature2D> orb = cv::ORB::create();
		orb -> detectAndCompute(image1, cv::Mat(), kpt1, des1);
		orb -> detectAndCompute(image2, cv::Mat(), kpt2, des2);
		orb -> detectAndCompute(image3, cv::Mat(), kpt3, des3);

		cv::Ptr<cv::DescriptorMatcher> matcher_orb = cv::BFMatcher::create(cv::NORM_HAMMING);
		std::vector<cv::DMatch> matches1, matches2;
		cv::Mat img1_img2_result, img2_img3_result;

		// ERASE THIS SENTENSE
		// Solve following description 
		// [OpenCV Error] Assertion failed ((globalDescIdx>=0) && (globalDescIdx < size())) in getLocalIdx
		if(kpt1.size() <= 2 || kpt2.size() <= 2 || kpt3.size() <= 2)
			return false;

        // OpenCV Error: Unsupported format or combination of formats (type=0) in buildIndex_,
        if(des1.empty() || des2.empty() || des3.empty())
        {
            std::cout << "Empty descriptor of image !!" << std::endl;
            return false;
        }

		matcher_orb -> match(des1, des2, matches1);
		matcher_orb -> match(des2, des3, matches2);
	// -------------------------------------------------------------------------

	// -------------------------------------------------------------------------
	/* [CASE 2] Feature Extraction and Feature Matching using SURF Method !!!
				SURF Feature Extraction + FLANN Matching using KnnMatch */
	#elif METHOD == 1
		cv::Mat des1, des2, des3;
		std::vector<cv::KeyPoint> kpt1, kpt2, kpt3;
		cv::Mat img1_img2_result, img2_img3_result;

		cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(300);
		surf -> detectAndCompute(image1, cv::Mat(), kpt1, des1);
		surf -> detectAndCompute(image2, cv::Mat(), kpt2, des2);
		surf -> detectAndCompute(image3, cv::Mat(), kpt3, des3);

		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
		std::vector<std::vector<cv::DMatch>> matches1, matches2;
		
		// ERASE THIS SENTENSE
		// Solve following description 
		// [OpenCV Error] Assertion failed ((globalDescIdx>=0) && (globalDescIdx < size())) in getLocalIdx
		if(kpt1.size() <= 2 || kpt2.size() <= 2 || kpt3.size() <= 2)
			return false;

        // OpenCV Error: Unsupported format or combination of formats (type=0) in buildIndex_,
        if(des1.empty() || des2.empty() || des3.empty())
        {
            std::cout << "Empty descriptor of image !!" << std::endl;
            return false;
        }

		matcher -> knnMatch(des1, des2, matches1, 2);
		matcher -> knnMatch(des2, des3, matches2, 2);
	// -------------------------------------------------------------------------

	// -------------------------------------------------------------------------
	/* [CASE 3] Feature Extraction and Feature Matching using SURF Method with MASKING !!!
				SURF Feature Extraction + FLANN Matching using KnnMatch */
	#elif METHOD == 2
		cv::Mat des1, des21, des22, des3;
		std::vector<cv::KeyPoint> kpt1, kpt21, kpt22, kpt3;
		cv::Mat img1_img2_result, img2_img3_result;

		cv::Mat white_img1(image1.rows, image1.cols, CV_8UC1, cv::Scalar(0, 0, 0));
		cv::Mat white_img21(image2.rows, image2.cols, CV_8UC1, cv::Scalar(0, 0, 0));
		cv::Mat white_img22(image2.rows, image2.cols, CV_8UC1, cv::Scalar(0, 0, 0));
		cv::Mat white_img3(image3.rows, image3.cols, CV_8UC1, cv::Scalar(0, 0, 0));

		cv::rectangle(white_img1, image1_rect, cv::Scalar(255, 255, 255), cv::FILLED, 8, 0);
		cv::rectangle(white_img21, image2_rect1, cv::Scalar(255, 255, 255), cv::FILLED, 8, 0);
		cv::rectangle(white_img22, image2_rect2, cv::Scalar(255, 255, 255), cv::FILLED, 8, 0);
		cv::rectangle(white_img3, image3_rect, cv::Scalar(255, 255, 255), cv::FILLED, 8, 0);

		cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(300);
		surf -> detectAndCompute(image1, white_img1, kpt1, des1);
		surf -> detectAndCompute(image2, white_img21, kpt21, des21);
		surf -> detectAndCompute(image2, white_img22, kpt22, des22);
		surf -> detectAndCompute(image3, white_img3, kpt3, des3);

		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
		std::vector<std::vector<cv::DMatch>> matches1, matches2;
		
		// ERASE THIS SENTENSE
		// Solve following description 
		// [OpenCV Error] Assertion failed ((globalDescIdx>=0) && (globalDescIdx < size())) in getLocalIdx
		if(kpt1.size() <= 2 || kpt21.size() <= 2 || kpt22.size() <= 2 || kpt3.size() <= 2)
			return false;

        // OpenCV Error: Unsupported format or combination of formats (type=0) in buildIndex_,
        if(des1.empty() || des21.empty() || des22.empty() || des3.empty())
        {
            std::cout << "Empty descriptor of image !!" << std::endl;
            return false;
        }

		matcher -> knnMatch(des1, des21, matches1, 2);
		matcher -> knnMatch(des22, des3, matches2, 2);
	// -------------------------------------------------------------------------
	#endif

    // -------------------------------------------------------------------------
	// Solve following description Part !
	// [OpenCV Error] Assertion failed (features1.descriptors.type() == features2.descriptors.type()) in match
	// -------------------------------------------------------------------------
	cv::Mat des2;
	std::vector<cv::KeyPoint> kpt2;
	surf -> detectAndCompute(image2, cv::Mat(), kpt2, des2);
	
	// images type is CV_8UC3 !! because the output is 16 !!
	cv::UMat img1_gray, img2_gray, img3_gray;
	cv::Mat clone_img1 = image1.clone();
	cv::Mat clone_img2 = image2.clone();
	cv::Mat clone_img3 = image3.clone();

	cv::cvtColor(clone_img1, img1_gray, cv::COLOR_BGR2GRAY);
	cv::cvtColor(clone_img2, img2_gray, cv::COLOR_BGR2GRAY);
	cv::cvtColor(clone_img3, img3_gray, cv::COLOR_BGR2GRAY);

	cv::UMat descriptor1, descriptor2, descriptor3;
	cv::detail::ImageFeatures feature1, feature2, feature3;

	surf->detectAndCompute(img1_gray, white_img1, feature1.keypoints, descriptor1);
	surf->detectAndCompute(img2_gray, cv::Mat(), feature2.keypoints, descriptor2);
	surf->detectAndCompute(img3_gray, white_img3, feature3.keypoints, descriptor3);

	feature1.descriptors = descriptor1.reshape(1, (int)feature1.keypoints.size());
	feature2.descriptors = descriptor2.reshape(1, (int)feature2.keypoints.size());
	feature3.descriptors = descriptor3.reshape(1, (int)feature3.keypoints.size());

	std::vector<std::vector<cv::DMatch>> feat_matches1, feat_matches2;
	matcher -> knnMatch(feature1.descriptors, feature2.descriptors, feat_matches1, 2);
	matcher -> knnMatch(feature2.descriptors, feature3.descriptors, feat_matches2, 2);

	cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
	cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SURF::create();

	cv::detail::ImageFeatures feat1, feat2, feat3;
	detector->detect(img1_gray, feat1.keypoints, white_img1);
	extractor->compute(img1_gray, feat1.keypoints, feat1.descriptors);
	detector->detect(img2_gray, feat2.keypoints);
	extractor->compute(img2_gray, feat2.keypoints, feat2.descriptors);
	detector->detect(img3_gray, feat3.keypoints, white_img3);
	extractor->compute(img3_gray, feat3.keypoints, feat3.descriptors);

	std::vector<std::vector<cv::DMatch>> feat_mat1, feat_mat2;

	matcher -> knnMatch(feat1.descriptors, feat2.descriptors, feat_mat1, 2);
	matcher -> knnMatch(feat2.descriptors, feat3.descriptors, feat_mat2, 2);

	std::cout << "To see des1 and des21 matching: " << feat_mat1.size() << std::endl;
	std::cout << "To see des22 and des3 matching: " << feat_mat2.size() << std::endl;
	// -------------------------------------------------------------------------
	// Final Solution of descriptor type error !!

	if(matches1.size() <= 10 || matches2.size() <= 10)
		return false;
	// -------------------------------------------------------------------------

    return true;
}
