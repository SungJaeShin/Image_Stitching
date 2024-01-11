#ifndef IMGPROC
#define IMGPROC

#include <opencv2/opencv.hpp>

/* [InterpolationFlags] 
Ref site: https://docs.opencv.org/3.2.0/da/d54/group__imgproc__transform.html#ga5bb5a1fea74ea38e1a5445ca803ff121

CV_INTER_NEAREST        : nearest neighbor interpolation
CV_INTER_LINEAR 	    : bilinear interpolation
CV_INTER_CUBIC 	        : bicubic interpolation
CV_INTER_AREA 	        : resampling using pixel area relation. 
                          It may be a preferred method for image decimation, as it gives moire'-free results. 
                          But when the image is zoomed, it is similar to the INTER_NEAREST method.
CV_INTER_LANCZOS4 	    : Lanczos interpolation over 8x8 neighborhood
CV_INTER_MAX 	        : mask for interpolation codes
CV_WARP_FILL_OUTLIERS 	: flag, fills all of the destination image pixels. 
                          If some of them correspond to outliers in the source image, they are set to zero
CV_WARP_INVERSE_MAP 	: flag, inverse transformation
*/

cv::Mat convertcvPolarimg(cv::Mat img)
{
    cv::Mat polar_img;
    cv::Point2f center( (float)img.cols / 2, (float)img.rows / 2 );
    double radius = (double)img.cols * 2;
    cv::linearPolar(img, polar_img, center, radius, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS);

    return polar_img;
}

cv::Mat convertcvLogPolarimg(cv::Mat img)
{
    cv::Mat log_polar_img;
    cv::Point2f center( (float)img.cols / 2, (float)img.rows / 2 );
    double radius = (double)img.cols * 2;
    double M = (double)img.cols / log(radius);
    cv::logPolar(img, log_polar_img, center, M, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS);
    
    return log_polar_img;
}

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

cv::Mat AddHomographyImg(cv::Mat img1, cv::Mat img2, cv::Mat warp_img2, cv::Mat H)
{
    double x_diff, y_diff;
    double x_min, y_min, x_max, y_max;
    cv::Mat ROIImg = calcROIImg(img2, H, x_diff, y_diff, x_min, y_min, x_max, y_max);

    // Translate img1 for warp img2 
    cv::Mat img1_trans = img1.clone();
    if(x_min <= 0 && y_min > 0)
    {
        cv::Mat x_trans_img1(img1.rows, (img1.cols - x_min), img1.type(), cv::Scalar(0,0,0));
        for(int y = 0; y < img1.rows; y++)
        {
            for(int x = 0; x < img1.cols; x++)
            {
                cv::Vec3b rgb = img1.at<cv::Vec3b>(y, x);
                x_trans_img1.at<cv::Vec3b>(y, x - x_min) = rgb;
            }
        }
        img1_trans = x_trans_img1.clone();
    }
    else if(x_min > 0 && y_min < 0)
    {
        cv::Mat y_trans_img1((img1.rows - y_min), img1.cols, img1.type(), cv::Scalar(0,0,0));
        for(int y = 0; y < img1.rows; y++)
        {
            for(int x = 0; x < img1.cols; x++)
            {
                cv::Vec3b rgb = img1.at<cv::Vec3b>(y, x);
                y_trans_img1.at<cv::Vec3b>(y - y_min, x) = rgb;
            }
        }
        img1_trans = y_trans_img1.clone();
    }
    else if(x_min < 0 && y_min < 0)
    {
        cv::Mat xy_trans_img1((img1.rows - y_min), (img1.cols - x_min), img1.type(), cv::Scalar(0,0,0));
        for(int y = 0; y < img1.rows; y++)
        {
            for(int x = 0; x < img1.cols; x++)
            {
                cv::Vec3b rgb = img1.at<cv::Vec3b>(y, x);
                xy_trans_img1.at<cv::Vec3b>(y - y_min, x - x_min) = rgb;
            }
        }
        img1_trans = xy_trans_img1.clone();
    }
    
    double img1_col = img1_trans.cols;
    double img1_row = img1_trans.rows;
    double img2_col = warp_img2.cols;
    double img2_row = warp_img2.rows;

    cv::Mat panoImg;
    if(img1_col <= img2_col && img1_row <= img2_row)
    {
        cv::Mat transImg(img2_row, img2_col, img1.type(), cv::Scalar(0,0,0));
        panoImg = transImg.clone();
    }
    else if(img1_col <= img2_col && img1_row > img2_row)
    {
        cv::Mat transImg(img1_row, img2_col, img1.type(), cv::Scalar(0,0,0));
        panoImg = transImg.clone();
    }
    else if(img1_col > img2_col && img1_row <= img2_row)
    {
        cv::Mat transImg(img2_row, img1_col, img1.type(), cv::Scalar(0,0,0));
        panoImg = transImg.clone();
    }
    else
    {
        cv::Mat transImg(img1_row, img1_col, img1.type(), cv::Scalar(0,0,0));
        panoImg = transImg.clone();
    }

    std::cout << "trans img size: " << img1_trans.size() << std::endl;
    std::cout << "pano img size: " << panoImg.size() << std::endl;

    // Add two imgs
    for(int y = 0; y < panoImg.rows; y++)
    {
        for(int x = 0; x < panoImg.cols; x++)
        {
            int final_r, final_g, final_b;
            cv::Vec3b compare(0, 0, 0);

            if(img1_col <= x && x < img2_col)
            {
                cv::Vec3b cur_rgb2 = warp_img2.at<cv::Vec3b>(y, x);
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
                cv::Vec3b cur_rgb1 = img1_trans.at<cv::Vec3b>(y, x);
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
                cv::Vec3b cur_rgb2 = warp_img2.at<cv::Vec3b>(y, x);
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
                cv::Vec3b cur_rgb1 = img1_trans.at<cv::Vec3b>(y, x);
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
            cv::Vec3b cur_rgb1 = img1_trans.at<cv::Vec3b>(y, x);
            cv::Vec3b cur_rgb2 = warp_img2.at<cv::Vec3b>(y, x);

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
