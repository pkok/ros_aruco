#include <ros_aruco/ArucoDetector.h>

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <ros/assert.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <aruco/aruco.h>

namespace std
{
  string to_string(const string& s) {
    return s;
  }
}

template <class T>
void getParamOrDie(const ros::NodeHandle& nh, const std::string& param_name, T& variable, bool verbose=false)
{
  if (nh.getParam(param_name, variable))
  {
    if (verbose)
      ROS_INFO("Got parameter \"%s\": %s",
          param_name.c_str(), std::to_string(variable).c_str());
  }
  else
  {
    ROS_ERROR("Failed to get parameter \"%s\"", param_name.c_str());
    ros::requestShutdown();
  }
}

namespace ros_aruco
{

ArucoDetector::ArucoDetector()
  : nh_("~"), it_(nh_), 
  MARKER_NS("aruco"), 
  MARKER_VISUALIZATION_TYPE(visualization_msgs::Marker::ARROW)
{
  std::string detector_topic, publish_topic;
  std::string image_color, camera_info, marker_size, output;
  getParamOrDie(nh_, "image_color", image_color);
  getParamOrDie(nh_, "camera_info", camera_info);
  getParamOrDie(nh_, "marker_size", (double&)marker_size_);
  getParamOrDie(nh_, "output", output);

  nh_.param("target_encoding", target_encoding_, std::string());

  MARKER_COLOR.r = 0.5;
  MARKER_COLOR.g = 0.5;
  MARKER_COLOR.b = 0.5;
  MARKER_COLOR.a = 0.5;

  // Subscribe to input video feed and publish output video feed
  camera_info_sub_ = nh_.subscribe(camera_info, 1,
      &ArucoDetector::cameraInfoCb, this);
  image_sub_ = it_.subscribe(image_color, 1,
      &ArucoDetector::detectorCb, this);
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(output, 100);
}


bool ArucoDetector::setCameraParameters(const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients, const cv::Size& size)
{
  if (distortionCoefficients.size().area() != 5
      || cameraMatrix.size().area() != 9)
    return false;
  try
  {
    camera_parameters_ = aruco::CameraParameters(cameraMatrix, distortionCoefficients, size);
  }
  catch (cv::Exception e)
  {
    return false;
  }
  camera_parameters_set_ = true;
  ROS_INFO("Camera parameters are set!");
  return true;
}


std::vector<aruco::Marker> ArucoDetector::detect(cv::Mat image)
{
  std::vector<aruco::Marker> markers;
  if (camera_parameters_set_)
    marker_detector_.detect(image, markers, camera_parameters_, marker_size_);
  return markers;
}


void ArucoDetector::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
{
  cv::Mat cameraMatrix(cv::Size(3,3), CV_64F, (void*) msg->K.elems);
  cv::Mat distortionCoefficients(msg->D, true);
  cv::Size size(msg->width, msg->height);
  ROS_ASSERT_MSG(msg->distortion_model == "plumb_bob", 
      "Only cameras with a 'plumb_bob' distortion model are supported");
  if (setCameraParameters(cameraMatrix, distortionCoefficients, size))
    camera_info_sub_.shutdown();
}


void ArucoDetector::detectorCb(const sensor_msgs::ImageConstPtr& msg)
{
#ifdef NDEBUG
  if (marker_pub_.getNumSubscribers() == 0 || !camera_parameters_set_)
    return;
#endif

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, target_encoding_);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  std::vector<aruco::Marker> detected_markers = detect(cv_ptr->image);
  if (detected_markers.empty())
    return;

#ifndef NDEBUG
  ROS_INFO("Markers detected");
#endif
  visualization_msgs::MarkerArray marker_msg;
  auto header = msg->header;

  for (aruco::Marker m : detected_markers) {
    visualization_msgs::Marker marker;
    m.calculateExtrinsics(marker_size_, camera_parameters_);
    double position[3];
    double orientation[4];

    m.OgreGetPoseParameters(position, orientation);

    marker.header = header;
    marker.ns = MARKER_NS;
    marker.type = visualization_msgs::Marker::CUBE;//MARKER_VISUALIZATION_TYPE;
    marker.color = MARKER_COLOR;
    marker.id = m.id;
    marker.lifetime = ros::Duration(0.1);
    marker.scale.x = 1.;
    marker.scale.y = .1;
    marker.scale.z = .1;
    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];
    marker.pose.orientation.w = orientation[0];
    marker.pose.orientation.x = orientation[1];
    marker.pose.orientation.y = orientation[2];
    marker.pose.orientation.z = orientation[3];

    marker_msg.markers.push_back(marker);
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
