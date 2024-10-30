Camera and LiDAR Synchronization and Visualization

This ROS 2 C++ package synchronizes camera and LiDAR data streams, applies transformations and projections to align the LiDAR points with the camera image, and visualizes them using intensity-based coloring. Synchronization is achieved with message filters using an approximate time policy.
Features

    Data Synchronization: Synchronizes sensor_msgs::Image and sensor_msgs::PointCloud2 topics using approximate time policy.
    LiDAR-Camera Projection: Transforms 3D LiDAR points to the camera frame and projects them onto the 2D image.
    Intensity-based Visualization: Visualizes projected LiDAR points on the camera image with intensity-based coloring.
    Filtering by Yaw: Filters LiDAR points within specified yaw angle limits before projecting them onto the camera image.
    Undistortion and Display: Applies camera undistortion and displays the final overlay of LiDAR points on the camera feed.

Dependencies

Ensure the following ROS 2 and external packages are installed:

    rclcpp
    sensor_msgs
    cv_bridge
    opencv
    yaml-cpp
    message_filters
    pcl_conversions
    pcl

Use your package manager (e.g., apt, vcpkg, or rosdep) to install any additional dependencies.
Code Structure

The main code sections are organized as follows:
Node Initialization

The CameraSubscriber class is derived from rclcpp::Node and subscribes to the camera and LiDAR topics.
YAML Calibration Loading

Loads camera calibration parameters from a YAML file, including the intrinsic matrix (camera_matrix) and distortion coefficients (distortion_coefficients).
Synchronization and Callbacks

    Synchronizes sensor_msgs::Image and sensor_msgs::PointCloud2 messages using message_filters.
    Defines a callback function to handle the processing of synchronized data.

Data Transformation and Filtering

    Projects 3D LiDAR points to 2D image points.
    Filters points within specified yaw limits.
    Undistorts points using camera calibration data.

Visualization

Visualizes LiDAR points on the undistorted image, with intensity-based coloring.
Usage

    Build and Launch the Node
    Compile and run the package using:

    bash

    colcon build --packages-select <package_name>
    source install/setup.bash
    ros2 run <package_name> <node_name>

    Parameter Customization
    Configure camera and LiDAR topics in the CameraSubscriber class.
        Adjust the path to the YAML file containing camera calibration data.
        Customize yaw limits, image resizing, and other visual settings in the code.

Function Overview

    filterPointsWithinYaw()
    Filters LiDAR points based on specified yaw angles in degrees.

    transformation()
    Applies a transformation matrix to the LiDAR points.

    projectLidarPoints()
    Projects 3D LiDAR points onto a 2D camera frame using camera intrinsic and extrinsic parameters.

    visualizeLidarPoints()
    Normalizes and applies a color map based on intensity, visualizing each LiDAR point on the image.

Example Configuration

Adjust the YAML path and topic names in the CameraSubscriber constructor as per your setup.

cpp

YAML::Node calib_data = YAML::LoadFile("<calibration_file_path>.yaml");
image_sub_.subscribe(this, "<camera_topic>", rmw_qos_profile_sensor_data);
pointcloud_sub_.subscribe(this, "<lidar_topic>", rmw_qos_profile_sensor_data);

License

This package is licensed under the MIT License.
Acknowledgments

This package leverages ROS 2, OpenCV, and PCL libraries to provide synchronized multi-sensor processing and visualization in robotics applications.
