#include <ros/ros.h>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/xfeatures2d.hpp>
// #include <opencv2/video/tracking.hpp>

#include <math.h>
#include <string>
#include <cmath>

// topics
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>

// variables from subscriber
namespace enc = sensor_msgs::image_encodings;
cv::Mat Img_raw;
cv::Mat Img_;
double width, height;

void bluefox_image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, enc::BGRA8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_ptr->image.copyTo(Img_raw);
    // cvtColor(Img_raw, Img_, cv::COLOR_BGR2GRAY);
    cvtColor(Img_raw, Img_, cv::COLOR_BGRA2RGB);
    // cv::imshow("Img_", Img_);
    // cv::waitKey(0);

    width = msg->width;
    height = msg->height;
    // print_info
    // cout << "height: " << msg->height << endl;
    // cout << "width: " << msg->width << endl;
    // cout << "step: " << msg->step << endl;
    // cout << "encoding: " << msg->encoding << endl;
    // cout << "data_size: " << msg->data.size() << endl;
    // cout << "---------------------" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mv_camera_calib");

    ros::NodeHandle nh;

    ros::Subscriber bluefox_image_sub = nh.subscribe("/0/image_raw", 1, bluefox_image_cb);
    
    // parameters
    cv::Size boardSize;
    double squareSize;
    if (!nh.getParam("/boardSize_width", boardSize.width))
        boardSize.width = 7;
    if (!nh.getParam("/boardSize_height", boardSize.height))
        boardSize.height = 6;
    if(!nh.getParam("/squareSize", squareSize))
        squareSize = 109.83; // in [mm]

    ros::Rate rate(1);

    // wait for camera data
    while (ros::ok())
    {
        if (0 < Img_.rows)
            break;
        
        ros::spinOnce();

        rate.sleep();
    }

    /*
    image storation
    */
    int save_num = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        std::string num = std::to_string(save_num);
        std::string file_name = "mv_mono" + num + ".jpg";
        cv::imwrite(file_name, Img_);

        cv::imshow("out", Img_);
        cv::waitKey(0);

        save_num++;

        rate.sleep();

        if (50 < save_num)
            break;
    }

    /*
    camera calibaration
    */
    while (ros::ok())
    {
        // detecting chessboard
        std::vector<cv::Point2f> corners; // output array of detected corners
        cv::Size ptSize(boardSize.width, boardSize.height);
        bool found = cv::findChessboardCorners( Img_, ptSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH );
        
        cv::drawChessboardCorners(Img_, ptSize, cv::Mat(corners), found);
        cv::imshow("grey image", Img_);


        // store checkerboard corners - image points
        std::vector<std::vector<cv::Point2f>> imagePoints(1); 
        for (int i = 0; i < corners.size(); ++i)
            imagePoints[0].push_back(corners[i]);

        // object points - defines the type of pattern
        std::vector<std::vector<cv::Point3f>> objectPoints(1);
        objectPoints[0].clear();
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                objectPoints[0].push_back(cv::Point3f(j*squareSize, i*squareSize, 0));

        double grid_width = squareSize * (boardSize.width - 1);
        objectPoints[0][boardSize.width-1].x = objectPoints[0][0].x + grid_width;

        std::vector<cv::Point3f> newObjPoints = objectPoints[0];
        
        objectPoints.resize(imagePoints.size(), objectPoints[0]);

        // std::cout << objectPoints[0].size() << std::endl;
        // std::cout << imagePoints[0].size() << std::endl;

        // int iFixedPoint = boardSize.width - 1;
        cv::Size imageSize = Img_.size();
        
        // outputs
        cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F); // output camera matrix
        cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F); // output distortion coefficients
        std::vector<cv::Mat> rvecs; // Output vector of rotation vectors (Rodrigues) estimated for each pattern view
        std::vector<cv::Mat> tvecs; // Output vector of translation vectors estimated for each pattern view 

        // calibration function
        double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, 
                                    cameraMatrix, distCoeffs, rvecs, tvecs);
        // double rms = cv::calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint, 
        //                             cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints, 
        //                             0 | cv::CALIB_USE_LU);
        
        cv::waitKey(0);
            
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
