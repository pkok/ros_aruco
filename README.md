ros_aruco
=========

ROS node that publishes the poses of detected ArUco markers.

The node can handle multiple markers at once.  It is inactive if no node is listening to its output topic.

---
Required parameters:

- `image_color`: topic where the uncalibrated RGB camera image is broadcast.
- `camera_info`: topic where the camera broadcasts its parameters.  At least `K` and `D` should be set.  The software expects that the `distortion_model` is `plumb_bob`.
- `marker_size`: size of a single side of the ArUco marker, in meters.
- `output`: topic where the `visualization_msgs/MarkerArray` will be posted.  


Some important values of each published `visualization_msgs/Marker` are:
  - `header`: copy of the image in which the marker is detected
  - `ns`: `ros_aruco::ArucoDetector::MARKER_NS = "aruco"`
  - `type`: `ros_aruco::ArucoDetector::MARKER_VISUALIZATION_TYPE = visualization_msgs::Marker::CUBE`
  - `color`: `ros_aruco::ArucoDetector::MARKER_COLOR` (50% gray with `alpha = 50`)
  - `id`: the detected marker's id
