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

    // Linear Interpolation pixels
    cv::Mat interImg = warpImg.clone();
    for(int y = 1; y < warpImg.rows - 1; y++)
    {
        for(int x = 1; x < warpImg.cols - 1; x++)
        {
            int cur_r = (int)warpImg.at<cv::Vec3b>(y, x)[0];
            int cur_g = (int)warpImg.at<cv::Vec3b>(y, x)[1];
            int cur_b = (int)warpImg.at<cv::Vec3b>(y, x)[2];

            int count = 0;
            for(int p = -1; p <= 1; p++)
            {
                for(int q = -1; q <= 1; q++)
                {
                    if(p == 0 && q == 0)
                        continue;
                    int tmp_r = (int)warpImg.at<cv::Vec3b>(y+p, x+q)[0];
                    int tmp_g = (int)warpImg.at<cv::Vec3b>(y+p, x+q)[1];
                    int tmp_b = (int)warpImg.at<cv::Vec3b>(y+p, x+q)[2];

                    if(tmp_r == 0 && tmp_g == 0 && tmp_b == 0)
                        count++;
                }
            }

            if(cur_r == 0 && cur_g == 0 && cur_b == 0)
            {
                if(count >= 0 && count < 7)
                {
                    bool minus = false;
                    bool plus = false; 

                    int final_inter_r, final_inter_g, final_inter_b;

                    cv::Vec3b pt1 = warpImg.at<cv::Vec3b>(y-1, x-1);
                    cv::Vec3b pt2 = warpImg.at<cv::Vec3b>(y+1, x+1);
                    cv::Vec3b inter_pt1 = LinearInter(pt1, pt2, 0.5);

                    cv::Vec3b pt3 = warpImg.at<cv::Vec3b>(y-1, x+1);
                    cv::Vec3b pt4 = warpImg.at<cv::Vec3b>(y+1, x-1);
                    cv::Vec3b inter_pt2 = LinearInter(pt3, pt4, 0.5);

                    cv::Vec3b compare(0, 0, 0);
                    if(pt1 == compare || pt2 == compare)
                    {
                        final_inter_r = (int)inter_pt2[0];
                        final_inter_g = (int)inter_pt2[1];
                        final_inter_b = (int)inter_pt2[2];
                    }
                    else if(pt3 == compare || pt4 == compare)
                    {
                        final_inter_r = (int)inter_pt1[0];
                        final_inter_g = (int)inter_pt1[1];
                        final_inter_b = (int)inter_pt1[2];
                    }
                    else
                    {
                        final_inter_r = ((int)inter_pt1[0] + (int)inter_pt2[0]) / 2;
                        final_inter_g = ((int)inter_pt1[1] + (int)inter_pt2[1]) / 2;
                        final_inter_b = ((int)inter_pt1[2] + (int)inter_pt2[2]) / 2;
                    }

                    cv::Vec3b final_inter_pt(final_inter_r, final_inter_g, final_inter_b);
                    interImg.at<cv::Vec3b>(y, x) = final_inter_pt;
                }
            }
        }
    }

    return interImg;
}