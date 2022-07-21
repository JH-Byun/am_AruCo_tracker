#include "ros/ros.h"
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "so3_utils.hpp"

// dynamixel
#include <sensor_msgs/JointState.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>
// camera calibarion
#include <opencv2/calib3d.hpp>
// ArUco detection
#include <opencv2/aruco.hpp>

#include <math.h>
#include <string>
#include <cmath>

// topics
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#define PI 3.141592
#define rad2deg 180 / 3.141592
#define deg2rad 3.141592 / 180
namespace enc = sensor_msgs::image_encodings;

/*
 variables from subscriber
*/
cv::Mat Img_raw;
cv::Mat Img_;
Eigen::Matrix<double, 3, 3> R_b; // rotation matrix of the multi-rotor
Eigen::Matrix<double, 2, 1> gma_, dgma_; // joint state subscribe
/*
parameters
*/
bool create_marker; // whether to create marker image file or not
std::vector<double> k_, p_; // elements of distortion coefficients
double fx, fy, cx, cy; // elements of camera matrix
bool vis_flag; // wherther to visualize the result or not

// global variables
std::vector<cv::Vec3d> tvecs;
std::vector<cv::Mat> R_cw_vec;

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
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    Eigen::Quaternion<double> q_b;
    q_b.w() = msg->orientation.w;
    q_b.x() = msg->orientation.x;
    q_b.y() = msg->orientation.y;
    q_b.z() = msg->orientation.z;

    R_b = q_b.toRotationMatrix();
}
void joint_state_cb(const sensor_msgs::JointState::ConstPtr &msg)
{
    gma_(0, 0) = deg2rad * (msg->position[0] - 180.0);
    gma_(1, 0) = deg2rad * (msg->position[1] - 180.0);
    dgma_(0, 0) = deg2rad * msg->velocity[0];
    dgma_(1, 0) = deg2rad * msg->velocity[1];

    // cout << "gma = " << gma.transpose() << endl;
    // cout << "gmadot = " << gmadot.transpose() << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mono_surface");

    ros::NodeHandle nh;

    ros::Subscriber bluefox_image_sub = nh.subscribe("/0/image_raw", 1, bluefox_image_cb);
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 1, imu_cb);
    ros::Subscriber joint_state_sub = nh.subscribe("/dynamixel/current", 1, joint_state_cb);
    ros::Publisher surface_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/surface/pose", 1);
    
    /*
     get parameters
     */
    if (!nh.getParam((ros::this_node::getName() + "/create_marker").c_str(), create_marker))
        create_marker = false;
    if (!nh.getParam((ros::this_node::getName() + "/k_").c_str(), k_))
        for (int i = 0; i < 6; ++i)
            k_.push_back(0.0);
    if (!nh.getParam((ros::this_node::getName() + "/p_").c_str(), p_))
        for (int i = 0; i < 2; ++i)
            p_.push_back(0.0);
    if (!nh.getParam((ros::this_node::getName() + "/fx").c_str(), fx))
        fx = 1.0;
    if (!nh.getParam((ros::this_node::getName() + "/fy").c_str(), fy))
        fy = 1.0;
    if (!nh.getParam((ros::this_node::getName() + "/cx").c_str(), cx))
        cx = 0.0;
    if (!nh.getParam((ros::this_node::getName() + 
        
        // orientation
        "/cy").c_str(), cy))
        cy = 0.0;
    if (!nh.getParam((ros::this_node::getName() + "/vis_flag").c_str(), vis_flag))
        vis_flag = false;

    /*
     set camera parameters
     */
    cv::Mat cameraMatrix, distCoeffs;
    cameraMatrix = (cv::Mat1d(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    distCoeffs = (cv::Mat1d(1, 4) << k_[0], k_[1], p_[0], p_[1]);
    distCoeffs = (cv::Mat1d(1, 5) << k_[0], k_[1], p_[0], p_[1], k_[2]);
    distCoeffs = (cv::Mat1d(1, 8) << k_[0], k_[1], p_[0], p_[1], k_[2], 
                                        k_[3], k_[4], k_[5]);

    // surface pose init.
    int cur_seq = 0;
    cv::Vec3d tvec(0.0, 0.0, 0.0);
    tvecs.push_back(tvec);
    R_cw_vec.push_back(cv::Mat::zeros(3, 3, CV_64F));        
    tvecs[0].val[0] = 0.0;
    tvecs[0].val[1] = 0.0;
    tvecs[0].val[2] = 0.0;
    R_cw_vec[0].at<double>(0,0) = -1.0;
    R_cw_vec[0].at<double>(0,1) = 0.0;
    R_cw_vec[0].at<double>(0,2) = 0.0;
    R_cw_vec[0].at<double>(1,0) = 0.0;
    R_cw_vec[0].at<double>(1,1) = 0.0;
    R_cw_vec[0].at<double>(1,2) = 1.0;
    R_cw_vec[0].at<double>(2,0) = 0.0;
    R_cw_vec[0].at<double>(2,1) = 0.0;
    R_cw_vec[0].at<double>(2,2) = -1.0;

    // initialize subscribing topic values
    R_b.setIdentity(); // rotation matrix of the multirotor
    gma_.setZero(), dgma_.setZero(); // set all dxl values to zero

    ros::Rate rate(1);

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
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    if (create_marker)
    {
        cv::Mat markerImage;
        cv::aruco::drawMarker(dictionary, 25, 200, markerImage, 1);

        cv::imshow("marker", markerImage);
        cv::imwrite("/home/jeonghyun/Downloads/marker25.png", markerImage);
        cv::waitKey(0);
        ROS_INFO("ArUco marker image file created...");
    }
    
    /*
     get surface pose information
     */ 
    while (ros::ok())
    {
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
         get rotation matrix from {W} to {C}
         */
        // std::cout << markerIds.size() << std::endl;
        // if at least one marker detected
        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(Img_copy, markerCorners, markerIds); // draw the detected ArUco marker
            std::vector<cv::Vec3d> rvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.07, cameraMatrix, distCoeffs, rvecs, tvecs); // estimate pose of the detected marker
            
            for (int i = 0; i < markerIds.size(); ++i) 
                cv::Rodrigues(rvecs[i], R_cw_vec[i]); // calculate rotation matrix (R_CW) from rvecs
        }
        // averaing process
        cv::Vec3d r_sum(0.0, 0.0, 0.0);
        for (int i = 0; i < R_cw_vec.size(); ++i)
        {
            cv::Vec3d r_cw_;
            cv::Rodrigues(R_cw_vec[i], r_cw_);

            r_sum = r_sum + r_cw_;
        }
        cv::Vec3d r_av_(0.0, 0.0, 0.0);
        r_av_ = (1.0 / double(R_cw_vec.size())) * r_sum;
        cv::Mat R_av_;
        cv::Rodrigues(r_av_, R_av_);
        
        Eigen::Matrix<double, 3, 3> R_cw;
        
        // orientation
        R_cw <<  R_av_.at<double>(0, 0), R_av_.at<double>(0, 1), R_av_.at<double>(0, 2),
            R_av_.at<double>(1, 0), R_av_.at<double>(1, 1), R_av_.at<double>(1, 2),
            R_av_.at<double>(2, 0), R_av_.at<double>(2, 1), R_av_.at<double>(2, 2);
        /*
         get rotation matrix from {C} to {B}
         */
        Eigen::Matrix<double, 3, 3> R_bc;
        R_bc = so3::rpy2R(0.0, gma_(1), gma_(0)) * so3::rpy2R(PI,0.0,PI);

        /*
         get rotation matrix of {W}
         */
        Eigen::Matrix<double, 3, 3> R_w = R_b * R_bc * R_cw;
        // std::cout << "R_w" << R_w << std::endl;
        // Eigen::Matrix<double, 3, 1> phi_ = so3::R2rpy(R_cw);
        // std::cout << "phi_: " << phi_ << std::endl;
        Eigen::Quaternion<double> q_w = so3::R2q(R_w);

        // std::cout << so3::rpy2R(0.0, gma_(1), gma_(0)) << std::endl;

        // cv::imshow("out", Img_copy);
        // cv::waitKey(0);

        // publish topic
        Eigen::Matrix<double, 3, 1> p_cw;
        p_cw << tvecs[0].val[0], tvecs[0].val[1], tvecs[0].val[2];
        // std::cout << "here" << std::endl;
        Eigen::Matrix<double, 1, 3> e_1, e_2, e_3;
        e_1 << 1.0, 0.0, 0.0;
        e_2 << 0.0, 1.0, 0.0;
        e_3 << 0.0, 0.0, 1.0;
        geometry_msgs::PoseStamped surface_pose;
        surface_pose.header.frame_id = "map";
        surface_pose.header.seq = cur_seq;
        surface_pose.header.stamp = ros::Time::now();
        surface_pose.pose.position.x = - e_1 * R_b * R_bc * p_cw;
        surface_pose.pose.position.y = - e_2 * R_b * R_bc * p_cw;
        surface_pose.pose.position.z = - e_3 * R_b * R_bc * p_cw;
        surface_pose.pose.orientation.w = q_w.w();
        surface_pose.pose.orientation.x = q_w.x();
        surface_pose.pose.orientation.y = q_w.y();
        surface_pose.pose.orientation.z = q_w.z();
        surface_pose_pub.publish(surface_pose);

        // update
        // prevImg = nextImg.clone();
        // prevPts.empty();
        // for (uint i = 0; i < nextPts.size(); ++i)
        // {
        //     prevPts.push_back(nextPts[i]);
        // }

        cur_seq++;

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
