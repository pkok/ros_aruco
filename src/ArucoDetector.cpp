#include <ros_aruco/ArucoDetector.h>
#include <ros_aruco/MarkerArray.h>

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <ros/assert.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <aruco/aruco.h>

namespace ros_aruco
{

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
  marker_pub_ = nh_.advertise<ros_aruco::MarkerArray>(publish_topic, 100);
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
  ros_aruco::MarkerArray marker_msg;
  marker_msg.header = msg->header;

  for (aruco::Marker m : markers) {
    ros_aruco::Marker marker;
    geometry_msgs::Pose pose;
    int16_t id;
    double position[3];
    double orientation[4];

    m.OgreGetPoseParameters(position, orientation);

    marker.pose.position.x = 2;// position[0];
    marker.pose.position.y = 2;// position[1];
    marker.pose.position.z = 2;// position[2];
    marker.pose.orientation.w = orientation[0];
    marker.pose.orientation.x = orientation[1];
    marker.pose.orientation.y = orientation[2];
    marker.pose.orientation.z = orientation[3];

    marker.id = m.id;

    marker_msg.push_back(marker);
  }
  marker_pub_.publish(marker_msg);
}

} // namespace ros_aruco


int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_detector");
  ros_aruco::ArucoDetector ic;
  ros::spin();
  return EXIT_SUCCESS;
}
