// stitching related
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/opencv.hpp"

#include "paramsetting.h"
#include "extractor.h"
#include "descriptor.h"
#include "outlier_rejection.h"
#include "warping.h"

bool opencv_based_stitching(std::vector<cv::Mat> imgs, 	std::vector<std::vector<cv::Rect>> masks,
                            std::vector<cv::detail::CameraParams> cams, cv::Mat &panorama)
{
	// Define mode and Make panorama image
	cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;
	cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode);
	
    if(FAST_STITCHING)
    {
        // *** [Feature Matcher] ***
        stitcher->setFeaturesMatcher(cv::makePtr<cv::detail::BestOf2NearestMatcher>(false));
        
        // // *** [Camera Estimator] Not in OpenCV 3.X***
        // cv::Ptr<cv::detail::Estimator> estimator = cv::makePtr<cv::detail::HomographyBasedEstimator>();
        // stitcher->setEstimator(estimator);

        // *** [Bundle Adjustment] ***
        stitcher->setBundleAdjuster(cv::makePtr<cv::detail::BundleAdjusterRay>());

        // *** [Warpping] ***
        stitcher->setWarper(cv::makePtr<cv::SphericalWarper>());
        // stitcher->setWarper(cv::makePtr<cv::PlaneWarper>());
        // stitcher->setWarper(cv::makePtr<cv::AffineWarper>());
        // stitcher->setWarper(cv::makePtr<cv::CylindricalWarper>());

        // *** [Exposure] ***
        stitcher->setExposureCompensator(cv::makePtr<cv::detail::GainCompensator>());
    
        // *** [Seam] ***
        stitcher->setSeamFinder(cv::makePtr<cv::detail::DpSeamFinder>(cv::detail::DpSeamFinder::COLOR));

        // *** [Blender] ***
        stitcher->setBlender(cv::makePtr<cv::detail::MultiBandBlender>(false));
    }

    #if MODE == 0
		// Original Method 
		cv::Stitcher::Status status = stitcher->stitch(imgs, panorama);
	#elif MODE == 1
		// Method 2 (with camera intrinsic)
		stitcher->cameras() = cams;
		cv::Stitcher::Status status = stitcher->stitch(imgs, panorama);
	#elif MODE == 2
		// Method 3 (With camera intrinsic + masking) 
		stitcher->cameras() = cams;
		cv::Stitcher::Status status = stitcher->stitch(imgs, masks, panorama);
	#elif MODE == 3
		// Method 4 (With camera intrinsic + masking) : Segmentation Fault & error: (-215) imgs.size() == imgs_.size() in function composePanorama
		stitcher->cameras() = cams;
		stitcher->estimateTransform(imgs, masks);
		cv::Stitcher::Status status = stitcher -> composePanorama(panorama);
	#endif

	if (status != cv::Stitcher::OK)
	{
		// Check if images could not be stiched
		// status is OK if images are stiched successfully
		std::cout <<  "Not Stitched !!!" << std::endl;
		return false;
	}

    return true;
}

bool homography_based_stitching(std::vector<cv::Mat> imgs, cv::Mat &panorama)
{
	// Initialization keypoints and descriptor 
    std::vector<cv::KeyPoint> img1_kpt, img2_kpt;
    std::vector<cv::Point2f> img1_2d, img2_2d;
    cv::Mat img1_des, img2_des;

    // Extract features and descriptors
    sift(imgs[0], img1_kpt);
    sift(imgs[1], img2_kpt);
    daisy(imgs[0], imgs[1], img1_kpt, img2_kpt, img1_des, img2_des);

	// Feature correspondence using features and descriptors
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2, false);
    std::vector<std::vector<cv::DMatch>> matches;
    matcher -> knnMatch(img1_des, img2_des, matches, 2);

	// Final correspondence pair
    cv::Mat F;
    std::vector<cv::KeyPoint> final_query_kpt, final_cand_kpt;
    std::vector<cv::DMatch> final_matches;
    F = outlierRejection(img1_2d, img2_2d, img1_kpt, img2_kpt, matches, 1, 0.999, 
                         final_query_kpt, final_cand_kpt, final_matches);

	// Set confidence like threshold
	double confidence = final_matches.size() / ((matches.size() * 0.3) + 8);
	if(confidence > 0.2)
    {
        // Estimate Homography using correspondence pair 
        cv::Mat H = cv::findHomography(img1_2d, img2_2d, CV_RANSAC, 1);
        std::cout << "H: " << H << std::endl;

        // Image Warping
        cv::Mat warp_img2 = convertWarpPlane(imgs[1], H);
        
        // Image Warping w/ CV
		cv::Mat warp_cvimg2 = convertcvWarpPlane(imgs[1], H);

        // Stitching Image
        cv::Mat panorama = AddcvImg(imgs[0], warp_cvimg2);
        

    }
    else
    {
        std::cout << "Not Stitch !!" << std::endl;
        return false; 
    }

	return true;
}