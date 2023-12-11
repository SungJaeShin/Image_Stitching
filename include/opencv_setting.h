#include "visualization.h"

// Ref site: https://github.com/opencv/opencv/blob/a2edf4d929ae22079dd1a301b88593e46e316256/modules/stitching/include/opencv2/stitching/detail/matchers.hpp
void initStitchingParam(cv::Mat img1, cv::Mat img1_des, cv::Mat img2, cv::Mat img2_des, 
                        std::vector<cv::KeyPoint> query_kpt, std::vector<cv::KeyPoint> cand_kpt,
                        std::vector<cv::DMatch> matcher, std::vector<cv::detail::ImageFeatures> &imgs_feat, 
                        cv::detail::MatchesInfo &cv_matches)
{
    // Set Image Features
    cv::detail::ImageFeatures cv_img1_feat, cv_img2_feat;

    cv_img1_feat.img_idx = 1;
	cv_img1_feat.img_size = img1.size();
	cv_img1_feat.keypoints = query_kpt;
	cv_img1_feat.descriptors = img1_des.getUMat(cv::ACCESS_READ);

	cv_img2_feat.img_idx = 2;
	cv_img2_feat.img_size = img2.size();
	cv_img2_feat.keypoints = cand_kpt;
	cv_img2_feat.descriptors = img2_des.getUMat(cv::ACCESS_READ);

    imgs_feat.push_back(cv_img1_feat);
	imgs_feat.push_back(cv_img2_feat);

    // Set Image Matchers
	cv_matches.src_img_idx = 1;
	cv_matches.dst_img_idx = 2;
	cv_matches.matches = matcher;
	cv_matches.num_inliers = matcher.size();    
}

void initCamParam(std::vector<cv::detail::CameraParams> &cv_cams)
{
    // Set Camera Parameters
	cv::detail::CameraParams cv_cam1, cv_cam2, cv_cam3;
    cv_cam1.R = (cv::Mat_<double>(3, 3) << 0.89594675, 0.00345519, 0.44414803, -0.44416133, 0.00777172, 0.89591312, -0.00035625, -0.99996383, 0.00849771);				
	cv_cam1.t = (cv::Mat_<double>(3, 1) << -0.00371292, 0.09689658, 0.00821615);
	
	cv_cam2.R = (cv::Mat_<double>(3, 3) << 0.00979306, 0.01637992, 0.99981788, -0.99995205, 0.00015501, 0.00979183, 0.00000541, -0.99986583, 0.01638065);
	cv_cam2.t = (cv::Mat_<double>(3, 1) << 0.07957005, 0.04586384, 0.06058584);
				
	cv_cam3.R = (cv::Mat_<double>(3, 3) << -0.90073466, 0.00384277, 0.43435274, -0.43436601, -0.00382317, -0.90072835, -0.00180068, -0.99998531, 0.00511283);
	cv_cam3.t = (cv::Mat_<double>(3, 1) << 0.08444155, -0.06144774, 0.00668201);

    cv_cams.push_back(cv_cam1);
	cv_cams.push_back(cv_cam2);
    cv_cams.push_back(cv_cam3);
}

void initMasking(cv::Mat image1, cv::Mat image2, cv::Mat image3,
                 std::vector<std::vector<cv::Rect>> &masks, std::vector<std::vector<cv::Rect>> &masks_multi)
{
    // Masking Related
	std::vector<cv::Rect> mask1;
	std::vector<cv::Rect> mask2;
	std::vector<cv::Rect> mask3;
	cv::Rect image1_rect(int(image1.cols/2), 0, int(image1.cols/2) , image1.rows);
	cv::Rect image2_rect(0, 0, image2.cols, image2.rows);
	cv::Rect image3_rect(0, 0, int(image3.cols/2), image3.rows);
	mask1.push_back(image1_rect);
	mask2.push_back(image2_rect);
	mask3.push_back(image3_rect);	
	masks.push_back(mask1);
	masks.push_back(mask2);
	masks.push_back(mask3);

	std::vector<cv::Rect> mask2_2;
	cv::Rect image2_rect1(0, 0, int(image2.cols/2), image2.rows);
	cv::Rect image2_rect2(int(image2.cols/2), 0, int(image2.cols/2), image2.rows);
	mask1.push_back(image1_rect);
	mask2_2.push_back(image2_rect1);
	mask2_2.push_back(image2_rect2);
	mask3.push_back(image3_rect);
	masks_multi.push_back(mask1);
	masks_multi.push_back(mask2_2);
	masks_multi.push_back(mask3);	

    if(SEE_MASKING)
        drawMasking(image1, image2, image3, image1_rect, image2_rect, image3_rect);
}
