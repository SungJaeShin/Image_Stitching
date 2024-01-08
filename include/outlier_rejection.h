#ifndef OUTLIER_REJECTION
#define OUTLIER_REJECTION

#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "parallax.h"

cv::Mat outlierRejection(std::vector<cv::Point2f> &img1_2d, std::vector<cv::Point2f> &img2_2d, 
                         std::vector<cv::KeyPoint> img1_kpt, std::vector<cv::KeyPoint> img2_kpt, 
                         std::vector<std::vector<cv::DMatch>> matches, double ransacReprojThreshold, double confidence,
                         std::vector<cv::KeyPoint> &final_query_kpt, std::vector<cv::KeyPoint> &final_cand_kpt,
                         std::vector<cv::DMatch> &final_matches)
{
    double threshold = 0.95f;
    std::vector<cv::KeyPoint> good_query_kpt, good_cand_kpt; 
	std::vector<cv::DMatch> good_matches;

    for(int i = 0; i < matches.size(); i ++)
    {
        if(matches[i][0].distance < threshold * matches[i][1].distance)
        {
            int query_idx = matches[i][0].queryIdx;
            int cand_idx = matches[i][0].trainIdx;
            img1_2d.push_back(img1_kpt[query_idx].pt);
            img2_2d.push_back(img2_kpt[cand_idx].pt);
            good_query_kpt.push_back(img1_kpt[query_idx]);
            good_cand_kpt.push_back(img2_kpt[cand_idx]);
            good_matches.push_back(matches[i][0]);
        }
    }

    // Get final correspondence pair     
    cv::Mat F;
    std::vector<uchar> status;
    F = cv::findFundamentalMat(img1_2d, img2_2d, cv::FM_RANSAC, ransacReprojThreshold, confidence, status);

    final_query_kpt = good_query_kpt;
    final_cand_kpt = good_cand_kpt;
    final_matches = good_matches;

    reduceVector(img1_2d, status);
    reduceVector(img2_2d, status);
    reduceVector(final_query_kpt, status);
    reduceVector(final_cand_kpt, status);
    reduceVector(final_matches, status);

    // // For Debugging
    // for(int i = 0; i < status.size(); i++)
    // {
    //     cv::Mat query_2pt = (cv::Mat_<double>(3, 1) << img1_2d[i].x, img1_2d[i].y, 1);
    //     cv::Mat cand_2pt = (cv::Mat_<double>(1, 3) << img2_2d[i].x, img2_2d[i].y, 1);
    //     if(status[i] > 0)
    //         std::cout << "[inlier] x'Fx = " << cand_2pt * F * query_2pt << std::endl;
    // }

    return F;
}

void outlierParallax(double avg_dx, double avg_dy,
                     std::vector<cv::KeyPoint> query_kpt, std::vector<cv::KeyPoint> cand_kpt, std::vector<cv::DMatch> matches,
                     std::vector<cv::KeyPoint> &final_query_kpt, std::vector<cv::KeyPoint> &final_cand_kpt, std::vector<cv::DMatch> &final_matches)
{
    for(size_t i = 0; i < matches.size(); i++)
    {
        double dx = std::fabs(query_kpt[i].pt.x - cand_kpt[i].pt.x);
        double dy = std::fabs(query_kpt[i].pt.y - cand_kpt[i].pt.y);

        if(dx < avg_dx - 10 || dx > avg_dx + 10)
            continue;
        if(dy < avg_dy - 10 || dy > avg_dy + 10)
            continue;

        final_query_kpt.push_back(query_kpt[i]);
        final_cand_kpt.push_back(cand_kpt[i]);
        final_matches.push_back(matches[i]);
    }
}
                         
#endif