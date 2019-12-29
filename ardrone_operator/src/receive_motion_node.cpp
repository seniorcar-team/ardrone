#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Header.h>

#include <Eigen/Dense>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <ardrone_operator/red_extraction.h>
#include <ardrone_operator/calc_area_in_contour.h>

#include <fstream>
#include <iostream>


cv::Mat track_image = cv::Mat::zeros(cv::Size(480,320), CV_8UC3);

bool first_flag = true;


class ReceiveMotion
{
public:
    ReceiveMotion();
    ~ReceiveMotion();
    void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);

private:
    ros::NodeHandle n_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher detect_image_pub_;
    image_transport::Publisher line_image_pub_;
    ros::Publisher cmd_pub_;
    std_msgs::Float32MultiArray cmd_msg_;
};


ReceiveMotion::ReceiveMotion()
{
    image_transport::ImageTransport it(n_);
    image_sub_ = it.subscribe("/image_raw", 1, &ReceiveMotion::imageCallback, this);
    detect_image_pub_ = it.advertise("/detect_image",10);
    line_image_pub_ = it.advertise("/line_image",10);
    cmd_pub_ = n_.advertise<std_msgs::Float32MultiArray>("/tum_ardrone/goto",50);
}

ReceiveMotion::~ReceiveMotion()
{
}

void ReceiveMotion::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    cv::Mat input_image;
    
    try
    {
        input_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    }
    //error
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Point2f image_center(input_image.cols / 2, input_image.rows / 2);

    cv::Mat extracted_image = RedExtraction::extractRedFrom(input_image);
    // double red_area = CalcAreaInContour::calcAreaInContour(extracted_image);
    cv::Point2f marker_centroid;
    bool is_detected = CalcAreaInContour::calcCentroidInContour(extracted_image, marker_centroid);

    std::vector<double> target_points(2.,0.);
    if(is_detected)
    {



        Eigen::Vector2d displacement = Eigen::Vector2d::Zero();
        displacement[0] = marker_centroid.x - image_center.x;
        displacement[1] = marker_centroid.y - image_center.y;
        std::cout << marker_centroid << std::endl;
        std::cout << displacement << std::endl;
        target_points[0] = displacement[0]/600.0;
        target_points[1] = -displacement[1]/600.0;
    }

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", extracted_image).toImageMsg();
    detect_image_pub_.publish(img_msg);
    cmd_msg_.data.resize(4);
    cmd_msg_.data[0] = 0.0;
    cmd_msg_.data[1] = target_points[0];
    cmd_msg_.data[2] = target_points[1];
    cmd_msg_.data[3] = 0.0;
    //cmd_pub_.publish(cmd_msg_);

    track_image.at<cv::Vec3b>(marker_centroid.y,marker_centroid.x)[0] = 50;
    track_image.at<cv::Vec3b>(marker_centroid.y,marker_centroid.x)[1] = 50;
    track_image.at<cv::Vec3b>(marker_centroid.y,marker_centroid.x)[2] = 255;

    std::string cont_x = std::to_string(target_points[0]);
    std::string cont_y = std::to_string(target_points[1]);
    std::ofstream flightfile("data.txt", std::ios::app);

    flightfile << "goto " << cont_x << " " << cont_y << " 0 0" << std::endl;

     sensor_msgs::ImagePtr line_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", track_image).toImageMsg();
    line_image_pub_.publish(line_img_msg);


    
}

int main(int argc, char** argv){
    ros::init (argc, argv, "receive_motion");
	ros::NodeHandle n;
    ros::Rate rate(10);

    if (first_flag == true){
        first_flag = false;

        std::ofstream flightfile("data.txt");

        flightfile << "autoInit 500 800 4000 0.5" << std::endl;

        std::ofstream flightfile_add("data.txt",std::ios::app);

        flightfile_add << "setReference $POSE$" << std::endl;
        flightfile_add << "setInitialReachDist 0.2" << std::endl;
        flightfile_add << "setStayWithinDist 0.3" << std::endl;
        flightfile_add << "setStayTime 3" << std::endl;
        flightfile_add << "lockScaleFP" << std::endl;

    }

    ReceiveMotion receive;

 	ros::spin();
 	return 0;
}