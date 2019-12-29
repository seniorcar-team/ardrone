#include <ardrone_operator/calc_area_in_contour.h>
#include <iostream>
#include <vector>

double CalcAreaInContour::calcAreaInContour(cv::Mat& image)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    double max_area = 0;
    int max_area_contour = -1;
    for(int j = 0; j < contours.size(); j++)
    {
        double area = contourArea(contours.at(j));
        if(max_area < area)
        {
            max_area = area;
            max_area_contour = j;
        }
    }
    return max_area;
}

bool CalcAreaInContour::calcCentroidInContour(cv::Mat& image, cv::Point2f& centroid)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    double max_area = 0;
    int max_id = 0;
    for(int k = 0; k < contours.size(); k++)
    {
        double area = contourArea(contours.at(k));
        if(max_area < area)
        {
            max_area = area;
            max_id = k;
        }
    }

    if (max_id  > 0)
    {
        cv::Moments mu = moments( contours[max_id]);
        centroid = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        //重心点の表示
        cv::circle( image, centroid, 4, cv::Scalar(100), 2, 4);
    }
    else
    {
        return false;
    }
    
    return true;
}



