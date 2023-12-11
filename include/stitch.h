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
        
        // *** [Camera Estimator] Not in OpenCV 3.X***
        cv::Ptr<cv::detail::Estimator> estimator = cv::makePtr<cv::detail::HomographyBasedEstimator>();
        stitcher->setEstimator(estimator);

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