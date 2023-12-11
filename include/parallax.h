#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "utility.h"
#include "paramsetting.h"

void computeOpticalFlow(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> &img1_2d, std::vector<cv::Point2f> &img2_2d,
                        std::vector<cv::KeyPoint> &final_query_kpt, std::vector<cv::KeyPoint> &final_cand_kpt, std::vector<cv::DMatch> &final_matches)
{
    if(img1_2d.size() != img2_2d.size())
    {
        std::cerr << "Difference size between img1 points and img2 points" << std::endl;
        return;
    }

    // Start Optical Flow
    // https://docs.opencv.org/3.4/dc/d6b/group__video__track.html#ga473e4b886d0bcc6b65831eb88ed93323
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(img1, img2, img1_2d, img2_2d, status, err, cv::Size(21, 21), 3);
    // cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

    // Reverse Check
    if(FLOW_BACK)
    {
        std::vector<uchar> reverse_status;
        cv::calcOpticalFlowPyrLK(img2, img1, img2_2d, img1_2d, reverse_status, err, cv::Size(21, 21), 3);
        for(size_t i = 0; i < status.size(); i++)
        {
            if(status[i] && reverse_status[i])
                status[i] = 1;
            else
                status[i] = 0;
        }
    }
    
    int success = 0;
    for(size_t i = 0; i < status.size(); i++)
        if(status[i])
            success++;
    
    std::cout << "\033[1;33m================== Optical Flow ==================\033[0m" << std::endl;
    std::cout << "Optical Flow Results: " << success << std::endl;

    reduceVector(img1_2d, status);
    reduceVector(img2_2d, status);
    reduceVector(final_query_kpt, status);
    reduceVector(final_cand_kpt, status);
    reduceVector(final_matches, status);
}

void computeParallax(double &avg_dx, double &avg_dy, double &avg_dist, 
                     double &max_dx, double &max_dy, double &max_dist,
                     std::vector<cv::KeyPoint> final_query_kpt, std::vector<cv::KeyPoint> final_cand_kpt,
                     std::vector<cv::DMatch> final_matches)
{
    double first_avg_dx = 0;
    double first_avg_dy = 0;
    double first_avg_dist = 0;
    double first_max_dx = 0;
    double first_max_dy = 0;
    double first_max_dist = 0;

    for(size_t i = 0; i < final_matches.size(); i++)
    {
        double dx = std::fabs(final_query_kpt[i].pt.x - final_cand_kpt[i].pt.x);
        double dy = std::fabs(final_query_kpt[i].pt.y - final_cand_kpt[i].pt.y);

        double dist = sqrt((dx * dx) + (dy * dy));

        first_max_dx = std::max(first_max_dx, dx);
        first_max_dy = std::max(first_max_dy, dy);
        first_max_dist = std::max(first_max_dist, dist);

        first_avg_dx = first_avg_dx + dx;
        first_avg_dy = first_avg_dy + dy;
        first_avg_dist = first_avg_dist + dist;
    }

    first_avg_dx /= final_matches.size();
    first_avg_dy /= final_matches.size();
    first_avg_dist /= final_matches.size();

    double count = 0;
    for(size_t i = 0; i < final_matches.size(); i++)
    {
        double dx = std::fabs(final_query_kpt[i].pt.x - final_cand_kpt[i].pt.x);
        double dy = std::fabs(final_query_kpt[i].pt.y - final_cand_kpt[i].pt.y);

        double dist = sqrt((dx * dx) + (dy * dy));

        if(dx < first_avg_dx - 10 || dx > first_avg_dx + 10)
            continue;
        if(dy < first_avg_dy - 10 || dy > first_avg_dy + 10)
            continue;

        max_dx = std::max(max_dx, dx);
        max_dy = std::max(max_dy, dy);
        max_dist = std::max(max_dist, dist);

        avg_dx = avg_dx + dx;
        avg_dy = avg_dy + dy;
        avg_dist = avg_dist + dist;
        count++;
    }

    avg_dx /= count;
    avg_dy /= count;
    avg_dist /= count;

    // // For Debugging
    // std::cout << "[Before Filtering]" << std::endl;
    // std::cout << "max dx: " << first_max_dx << ", max dy: " << first_max_dy << ", max dist: " << first_max_dist << std::endl;
    // std::cout << "avg dx: " << first_avg_dx << ", avg dy: " << first_avg_dy << ", avg dist: " << first_avg_dist << std::endl;
    
    // std::cout << "[After Filtering]" << std::endl;
    // std::cout << "inlier count: " << count << std::endl;
    // std::cout << "max dx: " << max_dx << ", max dy: " << max_dy << ", max dist: " << max_dist << std::endl;
    // std::cout << "avg dx: " << avg_dx << ", avg dy: " << avg_dy << ", avg_dist: " << avg_dist << std::endl;
}