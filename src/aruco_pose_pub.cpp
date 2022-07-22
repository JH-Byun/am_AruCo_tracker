// ROS
#include "ros/ros.h"
// print IO
#include <iostream>
#include <string>
// Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
// Header file for the functions related to SO3 operation
#include "so3_utils.hpp"
#include <cmath>
#include <math.h>
/*
 openCV related
 */
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <sensor_msgs/image_encodings.h>
// ArUco detection
#include <opencv2/aruco.hpp>
/*
 ROS topics
 */
// communication with mv_bluefox_USB2.0
#include <sensor_msgs/Image.h>
// communication with mavros (multicopter px4)
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
// obtain the pose of the attached camera w.r.t. the multicopter (px4)
// publish a topic about the pose of the detected AruCo marker
#include <geometry_msgs/PoseStamped.h>

/*
 definitions & namespaces
 */
#define PI 3.141592
#define rad2deg 180 / 3.141592
#define deg2rad 3.141592 / 180
namespace enc = sensor_msgs::image_encodings;

/*
 variables from subscriber
*/
cv::Mat Img_raw, Img_; // image data
Eigen::Matrix<double, 3, 1> p_b; // position of the multicopter
Eigen::Matrix<double, 3, 1> phi_b; // ZYX Euler angles of the multicopter
Eigen::Matrix<double, 3, 1> p_bc; // displacement of the camera w.r.t. multicopter
Eigen::Matrix<double, 3, 3> R_bc; // SO3 martrix of the camera w.r.t. multicopter
/*
parameter variables
*/
int rate_; // set publish rate
std::string dir_; // set directory of the marker storage
bool create_marker; // whether to create marker image file or not
int marker_type; // select the AruCo marker from the dictionary
std::string image_conv; // image conversion mode
std::vector<double> k_, p_; // elements of distortion coefficients
double fx, fy, cx, cy; // elements of camera matrix
double l_m; // length of the marker side
bool vis_flag; // wherther to visualize the result or not
/*
 variables for update
 */
int seq_; // time sequence

/*
 callback functions (cb) for ROS subscribers
 */
// cb for subscribing bluefox image data
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

    cv_ptr->image.copyTo(Img_raw); // get raw image data (encoder: BGRA8)

    // image conversion
    if (image_conv == "to_gray")
        cvtColor(Img_raw, Img_, cv::COLOR_BGR2GRAY);
    else if (image_conv == "to_rgb")
        cvtColor(Img_raw, Img_, cv::COLOR_BGRA2RGB);
    else {
        ROS_WARN("Error in the selection of image conversion mode!");
        ROS_WARN("Default setting: to_rgb");
        cvtColor(Img_raw, Img_, cv::COLOR_BGRA2RGB);
    }
}
// cb for subscribing multicopter's imu data
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    Eigen::Quaternion<double> q_b;
    q_b.w() = msg->orientation.w;
    q_b.x() = msg->orientation.x;
    q_b.y() = msg->orientation.y;
    q_b.z() = msg->orientation.z;

    Eigen::Matrix<double, 3, 3> R_b = q_b.toRotationMatrix();
    Eigen::Matrix<double, 3, 1> euler_rp = so3::R2rpy(R_b); // only roll and pitch values are valid
    phi_b(0) = euler_rp(0); // roll angle
    phi_b(1) = euler_rp(1); // pitch angle
}
// cb for subscribing multicopter's navigation data (position & yaw angle)
void nav_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // position
    p_b(0) = msg->pose.pose.position.x;
    p_b(1) = msg->pose.pose.position.y;
    p_b(2) = msg->pose.pose.position.z;

    // yaw angle
    Eigen::Quaternion<double> q_b;
    q_b.w() = msg->pose.pose.orientation.w;
    q_b.x() = msg->pose.pose.orientation.x;
    q_b.y() = msg->pose.pose.orientation.y;
    q_b.z() = msg->pose.pose.orientation.z;
    Eigen::Matrix<double, 3, 3> R_b = q_b.toRotationMatrix();
    Eigen::Matrix<double, 3, 1> euler_y = so3::R2rpy(R_b); // get only yaw value
    phi_b(2) = euler_y(2); // yaw angle
}
// cb for subscribing camera's pose w.r.t. multicopter
void camera_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // position
    p_bc(0) = msg->pose.position.x;
    p_bc(1) = msg->pose.position.y;
    p_bc(2) = msg->pose.position.z;

    // orientation
    Eigen::Quaternion<double> q_bc;
    q_bc.w() = msg->pose.orientation.w;
    q_bc.x() = msg->pose.orientation.x;
    q_bc.y() = msg->pose.orientation.y;
    q_bc.z() = msg->pose.orientation.z;
    R_bc = q_bc.toRotationMatrix();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_pose_pub");

    ros::NodeHandle nh;

    ros::Subscriber bluefox_image_sub = nh.subscribe("/0/image_raw", 1, bluefox_image_cb);
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 1, imu_cb);
    ros::Subscriber nav_pos_sub = nh.subscribe("/mavros/local_position/odom", 1, nav_pos_cb);
    ros::Subscriber camera_pose_sub = nh.subscribe("/body_to_camera", 1, camera_pose_cb);
    
    ros::Publisher marker_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/aruco_marker/pose", 1);
    
    /*
     get parameters
     */
    // set publish rate
    if (!nh.getParam((ros::this_node::getName() + "/publish_rate").c_str(), rate_))
        rate_ = 10;
    // create marker or not
    if (!nh.getParam((ros::this_node::getName() + "/create_marker").c_str(), create_marker))
        create_marker = false;
    // type of the aruco marker
    if (!nh.getParam((ros::this_node::getName() + "/marker_type").c_str(), marker_type))
        marker_type = 0;
    // set directory of the marker storage
    if (!nh.getParam((ros::this_node::getName() + "/store_directory").c_str(), dir_))
        dir_ = "";
    // set image conversion version ("to_gray" or "to_rgb")
    if (!nh.getParam((ros::this_node::getName() + "/image_conversion_version").c_str(), image_conv))
        image_conv = "to_rgb";
    /*
     set intrinsic (camera) matrix [[fx, 0, cx],[0, fy, cy],[0, 0, 1]]
     */
    if (!nh.getParam((ros::this_node::getName() + "/fx").c_str(), fx))
        fx = 1.0;
    if (!nh.getParam((ros::this_node::getName() + "/fy").c_str(), fy))
        fy = 1.0;
    if (!nh.getParam((ros::this_node::getName() + "/cx").c_str(), cx))
        cx = 0.0;
    if (!nh.getParam((ros::this_node::getName() + "/cy").c_str(), cy))
        cy = 0.0;
    cv::Mat cameraMatrix;
    cameraMatrix = (cv::Mat1d(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    /* 
     set distortion coefficients [k_1, k_2, k_3, k_4, k_5, k_6, p_1, p_2]
     */
    if (!nh.getParam((ros::this_node::getName() + "/k_").c_str(), k_))
        for (int i = 0; i < 6; ++i)
            k_.push_back(0.0);
    if (!nh.getParam((ros::this_node::getName() + "/p_").c_str(), p_))
        for (int i = 0; i < 2; ++i)
            p_.push_back(0.0);
    cv::Mat distCoeffs;
    distCoeffs = (cv::Mat1d(1, 4) << k_[0], k_[1], p_[0], p_[1]);
    distCoeffs = (cv::Mat1d(1, 5) << k_[0], k_[1], p_[0], p_[1], k_[2]);
    distCoeffs = (cv::Mat1d(1, 8) << k_[0], k_[1], p_[0], p_[1], k_[2], 
                                        k_[3], k_[4], k_[5]);
    // length of the marker side
    if (!nh.getParam((ros::this_node::getName() + "/side_length").c_str(), l_m))
        l_m = 0.05;
    // visualize the marker detection result or not
    if (!nh.getParam((ros::this_node::getName() + "/vis_flag").c_str(), vis_flag))
        vis_flag = false;

    /*
     initializing subscribing variables to avoid the error 
     occurred by no subcribing signal
     */
    p_b.setZero();
    phi_b.setZero();
    p_bc.setZero();
    R_bc << 0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0;
    bool flag_ = false; // initialization flag

    ros::Rate rate(rate_);

    // wait for camera data
    while (ros::ok())
    {
        if (0 < Img_.rows) {
            ROS_INFO("Camera data received...");
            break;
        }
            
        ros::spinOnce();

        rate.sleep();
    }

    /*
     create ArUco marker
     */
    // select the marker
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    switch ( marker_type )
    {
        case 0:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
            break;
        case 1:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            break;
        case 2:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_4X4_100);
            break;
        case 3:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_4X4_250);
            break;
        case 4:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
            break;
        case 5:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_5X5_50);
            break;
        case 6:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_5X5_100);
            break;
        case 7:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_5X5_250);
            break;
        case 8:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
            break;
        case 9:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_6X6_50);
            break;
        case 10:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_6X6_100);
            break;
        case 11:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            break;
        case 12:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
            break;
        case 13:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_7X7_50);
            break;
        case 14:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_7X7_100);
            break;
        case 15:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_7X7_250);
            break;
        case 16:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
            break;
        default:
            dictionary = getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
            break;
    }
    // create the marker file with the name "marker.jpg"
    if (create_marker)
    {
        cv::Mat markerImage;
        cv::aruco::drawMarker(dictionary, 25, 200, markerImage, 1);

        cv::imshow("marker", markerImage);
        std::string directory = dir_ + "marker.jpg";
        cv::imwrite(directory, markerImage);
        cv::waitKey(0);
        ROS_INFO("ArUco marker image file created...");
    }
    
    while (ros::ok())
    {
        // initialization
        if (flag_) {
            // time sequence setting
            seq_ = 0;
        }
        /*
         detect ArUco marker
         */
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(Img_, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        cv::Mat Img_copy;
        Img_.copyTo(Img_copy);

        /*
         get rotation matrix from {M} to {C}
         */
        std::vector<cv::Vec3d> tvecs; // vector of the displacement from {C} to {M}
        cv::Vec3d tvec(0.0, 0.0, 0.0);
        tvecs.push_back(tvec);
        std::vector<cv::Mat> R_cm_vec; // vector of the rotation matrix from {M} to {C}
        R_cm_vec.push_back(cv::Mat::zeros(3, 3, CV_64F));     
        // default value of R_cm
        R_cm_vec[0].at<double>(0,0) = 1.0, R_cm_vec[0].at<double>(0,1) = 0.0, R_cm_vec[0].at<double>(0,2) = 0.0;
        R_cm_vec[0].at<double>(1,0) = 0.0, R_cm_vec[0].at<double>(1,1) = -1.0, R_cm_vec[0].at<double>(1,2) = 0.0;
        R_cm_vec[0].at<double>(2,0) = 0.0, R_cm_vec[0].at<double>(2,1) = 0.0, R_cm_vec[0].at<double>(2,2) = -1.0;
        if (markerIds.size() > 0) { // if at least one marker detected
            cv::aruco::drawDetectedMarkers(Img_copy, markerCorners, markerIds); // draw the detected ArUco marker
            std::vector<cv::Vec3d> rvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, l_m, cameraMatrix, distCoeffs, rvecs, tvecs); // estimate pose of the detected marker
            
            for (int i = 0; i < markerIds.size(); ++i) 
                cv::Rodrigues(rvecs[i], R_cm_vec[i]); // calculate rotation matrix (R_CW) from rvecs
        }
        else
        {
            ROS_WARN("No AruCo marker detected!"); // Aruco marker detection failure!
        }
        /*
         averaging process
         */
        cv::Vec3d r_sum(0.0, 0.0, 0.0);
        for (int i = 0; i < R_cm_vec.size(); ++i)
        {
            cv::Vec3d r_cw_;
            cv::Rodrigues(R_cm_vec[i], r_cw_);

            r_sum = r_sum + r_cw_;
        }
        cv::Vec3d r_av_(0.0, 0.0, 0.0);
        r_av_ = (1.0 / double(R_cm_vec.size())) * r_sum;
        cv::Mat R_av_;
        cv::Rodrigues(r_av_, R_av_);
        Eigen::Matrix<double, 3, 3> R_cm;
        R_cm <<  R_av_.at<double>(0, 0), R_av_.at<double>(0, 1), R_av_.at<double>(0, 2),
            R_av_.at<double>(1, 0), R_av_.at<double>(1, 1), R_av_.at<double>(1, 2),
            R_av_.at<double>(2, 0), R_av_.at<double>(2, 1), R_av_.at<double>(2, 2);
        
        // calculate rotation matrix from {M} to {I}
        Eigen::Matrix<double, 3, 3> R_b = so3::rpy2R(phi_b(0), phi_b(1), phi_b(2));
        Eigen::Matrix<double, 3, 3> R_m = R_b * R_bc * R_cm * so3::rpy2R(0.0,PI/2,PI);

        // result visualizaition
        if (vis_flag) {
            cv::imshow("out", Img_copy);
            cv::waitKey(0);
        }
        
        // publish marker pose topic
        geometry_msgs::PoseStamped marker_pose;
        marker_pose.header.frame_id = "map";
        marker_pose.header.seq = seq_;
        marker_pose.header.stamp = ros::Time::now();
        Eigen::Matrix<double, 3, 1> p_cm;
        p_cm << tvecs[0].val[0], tvecs[0].val[1], tvecs[0].val[2];
        Eigen::Matrix<double, 3, 1> p_m = p_b + R_b * (p_bc + R_cm * p_cm);
        marker_pose.pose.position.x = p_m(0);
        marker_pose.pose.position.y = p_m(1);
        marker_pose.pose.position.z = p_m(2);
        Eigen::Quaternion<double> q_m = so3::R2q(R_m); // conversion from rotation matrix to quaternion
        marker_pose.pose.orientation.w = q_m.w();
        marker_pose.pose.orientation.x = q_m.x();
        marker_pose.pose.orientation.y = q_m.y();
        marker_pose.pose.orientation.z = q_m.z();
        marker_pose_pub.publish(marker_pose);

        // update time sequence
        seq_++;

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
