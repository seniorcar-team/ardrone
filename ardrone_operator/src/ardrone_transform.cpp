#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Header.h"

//点の情報宣言
//マーカー座標系におけるマーカーの座標
cv::Point3f p0 = cv::Point3f(0.0,0.0,0.0);
cv::Point3f p1 = cv::Point3f(0.0,0.0,0.0);
cv::Point3f p2 = cv::Point3f(0.0,0.0,0.0);
cv::Point3f p3 = cv::Point3f(0.0,0.0,0.0);
cv::Point3f p4 = cv::Point3f(0.0,0.0,0.0);
cv::Point3f p5 = cv::Point3f(0.0,0.0,0.0);

//画像座標系におけるマーカーの座標
cv::Point2d c0 = cv::Point2d(0.0,0.0);
cv::Point2d c1 = cv::Point2d(0.0,0.0);
cv::Point2d c2 = cv::Point2d(0.0,0.0);
cv::Point2d c3 = cv::Point2d(0.0,0.0);
cv::Point2d c4 = cv::Point2d(0.0,0.0);
cv::Point2d c5 = cv::Point2d(0.0,0.0);

//変換行列
//マーカーの座標郡
std::vector<cv::Point3f> marker_coordinate_points = {p0,p1,p2,p3,p4,p5};
//[マーカー座標系→カメラ座標系変換]行列
cv::Mat Rt_Matrix = ( cv::Mat_<double>( 4,4 ) << 1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0);
cv::Mat inv_Rt_Matrix = ( cv::Mat_<double>( 4,4 ) << 1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0);


///////////カメラの特性を表すパラメータ
//カメラ内部パラメータ
cv::Mat cameraMatrix = ( cv::Mat_<double>( 3,3 ) << 553.627733, 0.000000, 317.448667, 0.000000, 550.558377, 122.189254, 0.000000, 0.000000, 1.000000);
//歪み補正
cv::Mat distCoeffs =   (cv::Mat_<double>(5, 1) << -0.519086, 0.331704, 0.013667, 0.002975, 0.000000);







void imageCallback(const sensor_msgs::Image::ConstPtr& msg){

    //回転ベクトル
    cv::Mat r_vec = (cv::Mat_<double>(3,1) << 0.0,0.0,0.0);
    //並進ベクトル
    cv::Mat t_vec = (cv::Mat_<double>(3,1) << 0.0,0.0,0.0);
    //回転行列
    cv::Mat rMatrix = (cv::Mat_<double>(3,3) << 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);


    ////////////////////////////画像認識で点を認識するプログラム////////////////////////////////

        //get c0~c5 anyway

    ////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<cv::Point2d> camera_coordinate_points = {c0,c1,c2,c3,c4,c5};
    
    //solve p6p problem
    solvePnP(marker_coordinate_points,camera_coordinate_points, cameraMatrix, distCoeffs, r_vec, t_vec, false);

    //回転ベクトルから回転行列へ
    Rodrigues(r_vec, rMatrix);

    //座標変換ベクトルを作成
    for (int i=0; i<3; i++ ){
        for(int j=0; j<3; j++){

            Rt_Matrix.at<double>(i,j) = rMatrix.at<double>(i,j);
        }
    }
    for (int i=0; i<3; i++ ){
            Rt_Matrix.at<double>(i,3) = t_vec.at<double>(i,1);
    }


    //inv
    inv_Rt_Matrix = Rt_Matrix.inv();

}


int main(int argc, char** argv){
 	ros::init (argc, argv, "img_red");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw", 1, imageCallback);
        
    gray_image_pub = it.advertise("/image_nipple", 10);
	
 	ros::spin();
 	return 0;
}