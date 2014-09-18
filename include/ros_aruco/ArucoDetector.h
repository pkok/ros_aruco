#ifndef ROS_ARUCO_ARUCODETECTOR_H
#define ROS_ARUCO_ARUCODETECTOR_H

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <aruco/aruco.h>

class ArucoDetector
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher id_pub_;
    std::string target_encoding_;

    aruco::MarkerDetector marker_detector_;
    aruco::CameraParameters camera_parameters_;
    float marker_size_;

    ros::Publisher my_pub_;

  public:
    ArucoDetector();
    std::vector<aruco::Marker> detect(const cv::Mat image);

    void detectorCb(const sensor_msgs::ImageConstPtr& msg);
    void setMarkerSize(float marker_size);
};

#endif
