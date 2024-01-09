#ifndef IMGPROC
#define IMGPROC

#include <opencv2/opencv.hpp>

cv::Mat mean_filter(cv::Mat img)
{
    // Linear Interpolation
    cv::Mat interpolation_warpimg = img.clone();
    for(int y = 1; y < (img.rows - 1); y++)
    {
        for(int x = 1; x < (img.cols - 1); x++)
        {
            // Get RGB values
            int r = 0;
            int g = 0;
            int b = 0;
            for(int p = -1; p <= 1; p++)
            {
                for(int q = -1; q <= 1; q++)
                {
                    if(p == 0 && q == 0)
                        continue;
                    int tmp_r = (int)img.at<cv::Vec3b>(y+p, x+q)[0];
                    int tmp_g = (int)img.at<cv::Vec3b>(y+p, x+q)[1];
                    int tmp_b = (int)img.at<cv::Vec3b>(y+p, x+q)[2];

                    r+=tmp_r;
                    g+=tmp_g;
                    b+=tmp_b;
                }
            }

            r /= 8;
            g /= 8;
            b /= 8; 

            cv::Vec3b rgb(r, g, b);
            std::cout << "rgb: " << rgb << std::endl;
            interpolation_warpimg.at<cv::Vec3b>(y, x) = rgb;
        }
    }
    
    return interpolation_warpimg;
}

cv::Vec3b PixelLinearInter(cv::Vec3b pt1, cv::Vec3b pt2, double t)
{
    int interpolation_r = (int)pt1[0] + t * ((int)pt2[0] - (int)pt1[0]);
    int interpolation_g = (int)pt1[1] + t * ((int)pt2[1] - (int)pt1[1]);
    int interpolation_b = (int)pt1[2] + t * ((int)pt2[2] - (int)pt1[2]);
    cv::Vec3b inter_rgb(interpolation_r, interpolation_g, interpolation_b);

    return inter_rgb;
}

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

    // Make ROIimg size (called ROI img)
    x_min = std::min({H_pt1.at<double>(0,0), H_pt2.at<double>(0,0), H_pt3.at<double>(0,0), H_pt4.at<double>(0,0)});
    x_max = std::max({H_pt1.at<double>(0,0), H_pt2.at<double>(0,0), H_pt3.at<double>(0,0), H_pt4.at<double>(0,0)});
    y_min = std::min({H_pt1.at<double>(1,0), H_pt2.at<double>(1,0), H_pt3.at<double>(1,0), H_pt4.at<double>(1,0)});
    y_max = std::max({H_pt1.at<double>(1,0), H_pt2.at<double>(1,0), H_pt3.at<double>(1,0), H_pt4.at<double>(1,0)});
    
    x_diff = std::fabs(x_max - x_min);
    y_diff = std::fabs(y_max - y_min);
    cv::Mat ROIimg(y_diff, x_diff, img.type(), cv::Scalar(0,0,0));

    return ROIimg;
}

cv::Mat AddcvImg(cv::Mat img1, cv::Mat img2)
{
    double img1_col = img1.cols;
    double img1_row = img1.rows;
    double img2_col = img2.cols;
    double img2_row = img2.rows;

    cv::Mat panoImg;
    if(img1_col <= img2_col && img1_row <= img2_row)
    {
        cv::Mat warpImg(img2_row, img2_col, img1.type(), cv::Scalar(0,0,0));
        panoImg = warpImg.clone();
    }
    else if(img1_col <= img2_col && img1_row > img2_row)
    {
        cv::Mat warpImg(img1_row, img2_col, img1.type(), cv::Scalar(0,0,0));
        panoImg = warpImg.clone();
    }
    else if(img1_col > img2_col && img1_row <= img2_row)
    {
        cv::Mat warpImg(img2_row, img1_col, img1.type(), cv::Scalar(0,0,0));
        panoImg = warpImg.clone();
    }
    else
    {
        cv::Mat warpImg(img1_row, img1_col, img1.type(), cv::Scalar(0,0,0));
        panoImg = warpImg.clone();
    }

    // Add two imgs
    for(int y = 0; y < panoImg.rows; y++)
    {
        for(int x = 0; x < panoImg.cols; x++)
        {
            int final_r, final_g, final_b;
            cv::Vec3b compare(0, 0, 0);

            if(img1_col <= x && x < img2_col)
            {
                cv::Vec3b cur_rgb2 = img2.at<cv::Vec3b>(y, x);
                if(cur_rgb2 == compare)
                    continue;
                else
                {
                    final_r = (int)cur_rgb2[0];
                    final_g = (int)cur_rgb2[1];
                    final_b = (int)cur_rgb2[2];
                }
                
                cv::Vec3b final_pt(final_r, final_g, final_b);
                panoImg.at<cv::Vec3b>(y, x) = final_pt;

                continue;
            }
            else if(img2_col <= x && x < img1_col)
            {
                cv::Vec3b cur_rgb1 = img1.at<cv::Vec3b>(y, x);
                if(cur_rgb1 == compare)
                    continue;
                else
                {
                    final_r = (int)cur_rgb1[0];
                    final_g = (int)cur_rgb1[1];
                    final_b = (int)cur_rgb1[2];
                }
                
                cv::Vec3b final_pt(final_r, final_g, final_b);
                panoImg.at<cv::Vec3b>(y, x) = final_pt;

                continue;
            }    

            if(img1_row <= y && y < img2_row)
            {
                cv::Vec3b cur_rgb2 = img2.at<cv::Vec3b>(y, x);
                if(cur_rgb2 == compare)
                    continue;
                else
                {
                    final_r = (int)cur_rgb2[0];
                    final_g = (int)cur_rgb2[1];
                    final_b = (int)cur_rgb2[2];
                }
                
                cv::Vec3b final_pt(final_r, final_g, final_b);
                panoImg.at<cv::Vec3b>(y, x) = final_pt;

                continue;
            }
            else if(img2_row <= y && y < img1_row)
            {
                cv::Vec3b cur_rgb1 = img1.at<cv::Vec3b>(y, x);
                if(cur_rgb1 == compare)
                    continue;
                else
                {
                    final_r = (int)cur_rgb1[0];
                    final_g = (int)cur_rgb1[1];
                    final_b = (int)cur_rgb1[2];
                }
                
                cv::Vec3b final_pt(final_r, final_g, final_b);
                panoImg.at<cv::Vec3b>(y, x) = final_pt;

                continue;
            }

            // Get RGB values 
            cv::Vec3b cur_rgb1 = img1.at<cv::Vec3b>(y, x);
            cv::Vec3b cur_rgb2 = img2.at<cv::Vec3b>(y, x);

            if(cur_rgb1 == compare && cur_rgb2 == compare)
                continue;
            else if(cur_rgb1 == compare && cur_rgb2 != compare)
            {
                final_r = (int)cur_rgb2[0];
                final_g = (int)cur_rgb2[1];
                final_b = (int)cur_rgb2[2];
            }
            else if(cur_rgb1 != compare && cur_rgb2 == compare)
            {
                final_r = (int)cur_rgb1[0];
                final_g = (int)cur_rgb1[1];
                final_b = (int)cur_rgb1[2];
            }
            else
            {
                final_r = ((int)cur_rgb1[0] + (int)cur_rgb2[0]) / 2;
                final_g = ((int)cur_rgb1[1] + (int)cur_rgb2[1]) / 2;
                final_b = ((int)cur_rgb1[2] + (int)cur_rgb2[2]) / 2;
            }

            cv::Vec3b final_pt(final_r, final_g, final_b);
            panoImg.at<cv::Vec3b>(y, x) = final_pt;
        }
    }

    return panoImg;
}

#endif
