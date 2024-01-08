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

cv::Mat calcROIImg(cv::Mat img, cv::Mat H, double &x_diff, double &y_diff,
                   double &x_min, double &y_min, double &x_max, double &y_max)
{
    // Get warp corner points
    cv::Mat pt1(3,1,cv::DataType<double>::type);
    pt1.at<double>(0,0) = 0;
    pt1.at<double>(1,0) = 0;
    pt1.at<double>(2,0) = 1;
    cv::Mat H_pt1 = H.inv() * pt1;
    H_pt1 = H_pt1 / H_pt1.at<double>(2,0);

    cv::Mat pt2(3,1,cv::DataType<double>::type);
    pt2.at<double>(0,0) = img.cols;
    pt2.at<double>(1,0) = 0;
    pt2.at<double>(2,0) = 1;
    cv::Mat H_pt2 = H.inv() * pt2;
    H_pt2 = H_pt2 / H_pt2.at<double>(2,0);

    cv::Mat pt3(3,1,cv::DataType<double>::type);
    pt3.at<double>(0,0) = 0;
    pt3.at<double>(1,0) = img.rows;
    pt3.at<double>(2,0) = 1;
    cv::Mat H_pt3 = H.inv() * pt3;
    H_pt3 = H_pt3 / H_pt3.at<double>(2,0);

    cv::Mat pt4(3,1,cv::DataType<double>::type);
    pt4.at<double>(0,0) = img.cols;
    pt4.at<double>(1,0) = img.rows;
    pt4.at<double>(2,0) = 1;
    cv::Mat H_pt4 = H.inv() * pt4;
    H_pt4 = H_pt4 / H_pt4.at<double>(2,0);

    // Make WarpImg size (called ROI img)
    x_min = std::min({H_pt1.at<double>(0,0), H_pt2.at<double>(0,0), H_pt3.at<double>(0,0), H_pt4.at<double>(0,0)});
    x_max = std::max({H_pt1.at<double>(0,0), H_pt2.at<double>(0,0), H_pt3.at<double>(0,0), H_pt4.at<double>(0,0)});
    y_min = std::min({H_pt1.at<double>(1,0), H_pt2.at<double>(1,0), H_pt3.at<double>(1,0), H_pt4.at<double>(1,0)});
    y_max = std::max({H_pt1.at<double>(1,0), H_pt2.at<double>(1,0), H_pt3.at<double>(1,0), H_pt4.at<double>(1,0)});
    
    x_diff = std::fabs(x_max - x_min);
    y_diff = std::fabs(y_max - y_min);
    cv::Mat warpImg(y_diff, x_diff, img.type(), cv::Scalar(0,0,0));

    return warpImg;
}

cv::Mat convertcvWarpPlane(cv::Mat img, cv::Mat H)
{
    double x_diff, y_diff;
    double x_min, y_min, x_max, y_max;
    cv::Mat warpImg = calcROIImg(img, H, x_diff, y_diff, x_min, y_min, x_max, y_max);

    // CV_INTER_CUBIC CV_INTER_LINEAR  
    cv::warpPerspective(img, warpImg, H.inv(), cv::Size(x_diff, y_diff), CV_INTER_LINEAR, cv::BORDER_CONSTANT);

    return warpImg;
}

// Not add CV_Linear_Interpolation
cv::Mat convertWarpPlane(cv::Mat img, cv::Mat H)
{
    double x_diff, y_diff;
    double x_min, y_min, x_max, y_max;
    cv::Mat warpImg = calcROIImg(img, H, x_diff, y_diff, x_min, y_min, x_max, y_max);

    // WarpImg pixels
    for(int y = 0; y < img.rows; y++)
    {
        for(int x = 0; x < img.cols; x++)
        {
            // Get RGB values
            cv::Vec3b rgb = img.at<cv::Vec3b>(y, x);

            cv::Mat tmp_point(3,1,cv::DataType<double>::type);
            tmp_point.at<double>(0,0) = x;
            tmp_point.at<double>(1,0) = y;
            tmp_point.at<double>(2,0) = 1;

            cv::Mat H_tmp_point = H.inv() * tmp_point;
            H_tmp_point = H_tmp_point / H_tmp_point.at<double>(2,0);
            H_tmp_point.at<double>(0,0) = H_tmp_point.at<double>(0,0) - x_min;
            H_tmp_point.at<double>(1,0) = H_tmp_point.at<double>(1,0) - y_min;
        
            warpImg.at<cv::Vec3b>(H_tmp_point.at<double>(1,0), H_tmp_point.at<double>(0,0)) = rgb;
        }
    }

    return warpImg;
}