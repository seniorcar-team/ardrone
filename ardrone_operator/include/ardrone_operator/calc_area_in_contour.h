#ifndef CALC_AREA_IN_CONTOUR_H_
#define CALC_AREA_IN_CONTOUR_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <vector>

class CalcAreaInContour
{
public:
    static double calcAreaInContour(cv::Mat& image);
    static bool calcCentroidInContour(cv::Mat& image, cv::Point2f& centroid);
};

#endif // CALC_AREA_IN_CONTOUR_H_