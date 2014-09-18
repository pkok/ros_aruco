#include <ros_aruco/ArucoDetector.h>

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <ros/assert.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#include <aruco/aruco.h>

ArucoDetector::ArucoDetector()
  : nh_("~"), it_(nh_), 
    camera_parameters_(cv::Mat::eye(3, 3, CV_32F), 
                       cv::Mat::zeros(4, 1, CV_32F), 
                       cv::Size(640,480))
{
  std::string detector_topic, publish_topic;
  if (nh_.getParam("detect", detector_topic)) {
    ROS_INFO("Got parameter \"detect\": %s", detector_topic.c_str());
  }
  else {
    ROS_ERROR("Failed to get parameter \"detect\"");
    exit(EXIT_FAILURE);
  }

  if (nh_.getParam("publish", publish_topic)) {
    ROS_INFO("Got parameter \"publish\": %s", publish_topic.c_str());
  }
  else {
    ROS_ERROR("Failed to get parameter \"publish\"");
    exit(EXIT_FAILURE);
  }

  if (nh_.getParam("marker_size", (double&)marker_size_)) {
    ROS_INFO("Got parameter \"marker_size\": %s", std::to_string(marker_size_).c_str());
  }
  else {
    ROS_ERROR("Failed to get parameter \"marker_size\"");
    exit(EXIT_FAILURE);
  }

  nh_.param("target_encoding", target_encoding_, std::string());

  // Subscribe to input video feed and publish output video feed
  image_sub_ = it_.subscribe(detector_topic, 1,
    &ArucoDetector::detectorCb, this);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>(publish_topic + "/markers", 1);
  my_pub_ = nh_.advertise<geometry_msgs::Pose>(publish_topic + "/pose", 1);
  id_pub_ = nh_.advertise<std_msgs::Int16MultiArray>(publish_topic + "/markers/ids", 1);
}


std::vector<aruco::Marker> ArucoDetector::detect(cv::Mat image)
{
  std::vector<aruco::Marker> markers;
  camera_parameters_.resize(image.size());
  marker_detector_.detect(image, markers, camera_parameters_, marker_size_);
  /*
  for (auto marker : markers) {
    marker.calculateExtrinsics(marker_size_, camera_parameters_);
  }
  */
  return markers;
}


void ArucoDetector::detectorCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, target_encoding_);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<aruco::Marker> markers = detect(cv_ptr->image);

  geometry_msgs::PoseArray poses;
  std_msgs::Int16MultiArray ids;
  poses.header = msg->header;

  for (aruco::Marker marker : markers) {
    geometry_msgs::Pose pose;
    int16_t id;
    double position[3];
    double orientation[4];

    marker.OgreGetPoseParameters(position, orientation);

    pose.position.x = 2;// position[0];
    pose.position.y = 2;// position[1];
    pose.position.z = 2;// position[2];
    pose.orientation.w = orientation[0];
    pose.orientation.x = orientation[1];
    pose.orientation.y = orientation[2];
    pose.orientation.z = orientation[3];

    id = marker.id;

    poses.poses.push_back(pose);
    ids.data.push_back(id);

    my_pub_.publish(pose);
    break;
  }

  id_pub_.publish(ids);
  if (!poses.poses.empty() && !ids.data.empty())
  {
    pose_pub_.publish(poses);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ArucoDetector ic;
  ros::spin();
  return EXIT_SUCCESS;
}
