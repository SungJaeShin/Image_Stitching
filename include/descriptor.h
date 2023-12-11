#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"

void surf(cv::Mat image1, cv::Mat image2, std::vector<cv::KeyPoint> &img1_kpt, std::vector<cv::KeyPoint> &img2_kpt, cv::Mat &img1_des, cv::Mat &img2_des)
{
    // Feature Descriptor
    /* Must 'Hessian Threshold' that meaning how large the output from the Hessian filter must be in order for a point to be used as an interest point
       [-] Reference Site: https://stackoverflow.com/questions/18744051/what-the-hessian-minimum-threshold-means-on-the-surffeaturedetector-function 
       [-] Reference Site: https://www.researchgate.net/figure/Relation-between-hessian-threshold-value-and-Accuracy-of-SURF-detector-As-shown-in_fig2_309363445 */
	cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(500);
    surf -> compute(image1, img1_kpt, img1_des);
    surf -> compute(image2, img2_kpt, img2_des);   
}

void daisy(cv::Mat image1, cv::Mat image2, std::vector<cv::KeyPoint> &img1_kpt, std::vector<cv::KeyPoint> &img2_kpt, cv::Mat &img1_des, cv::Mat &img2_des)
{
    // Feature Descriptor
    /* Must 'Hessian Threshold' that meaning how large the output from the Hessian filter must be in order for a point to be used as an interest point
       [-] Reference Site: https://docs.opencv.org/3.4/d9/d37/classcv_1_1xfeatures2d_1_1DAISY.html */
	cv::Ptr<cv::xfeatures2d::DAISY> daisy = cv::xfeatures2d::DAISY::create();
    daisy -> compute(image1, img1_kpt, img1_des);
    daisy -> compute(image2, img2_kpt, img2_des);   
}