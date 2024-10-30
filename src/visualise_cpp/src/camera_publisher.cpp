#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>


class CameraSubscriber : public rclcpp::Node
{
        //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> MySyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
        cv::Mat matrix_coefficients_mat;
        cv::Mat distortion_coefficients_mat;
        cv::Mat resized_image;
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;  // Using XYZ + Intensity
        std::vector<std::array<float, 3>> points; 
        std::vector<float> point_intensity;

        // Initialize translation vector (tvec)
        cv::Mat tvec = (cv::Mat_<double>(3, 1) << -0.12755348, 0.03376902, 0.09379576); // Lidar 1108 and camera 618

        // Initialize rotation vector (rvec)
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << 0.82430635, -1.5835378, 1.71078744); // Lidar 1108 and camera 618
public:
    CameraSubscriber()
        : Node("camera_subscriber")
    {

        YAML::Node calib_data = YAML::LoadFile("/home/abd1340m/Dokumente/extrinsic_calibration/calibration_results/1108_618_data/40243618.yaml");
        std::vector<double> matrix_coefficients = calib_data["camera_matrix"]["data"].as<std::vector<double>>();
        std::vector<double> distortion_coefficients = calib_data["distortion_coefficients"]["data"].as<std::vector<double>>();

        
        matrix_coefficients_mat = cv::Mat(3, 3, CV_64F, matrix_coefficients.data()).clone(); // Use CV_32F for float
        distortion_coefficients_mat = cv::Mat(distortion_coefficients.size(), 1, CV_64F, distortion_coefficients.data()).clone(); // Use CV_32F for float


        image_sub_.subscribe(this, "/basler_pole_a_right_id_104_sn_618/my_camera/pylon_ros2_camera_node/image_raw", rmw_qos_profile_sensor_data);
        pointcloud_sub_.subscribe(this, "/ouster_pole_a_1108/points", rmw_qos_profile_sensor_data);


        auto sync_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>(20);
        sync_policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.3)); // Set the slop to 0.05 seconds
        //sync_policy.setInterMessageLowerBound(0, rclcpp::Duration(std::chrono::milliseconds(100)));
        
        sync_ = std::make_shared<message_filters::Synchronizer<decltype(sync_policy)>>(sync_policy);
        sync_->connectInput(image_sub_, pointcloud_sub_);
        sync_->registerCallback(std::bind(&CameraSubscriber::Callback, this, std::placeholders::_1, std::placeholders::_2));
        
    }

    std::vector<std::array<float, 3>> transformation(const std::vector<std::array<float, 3>>& pc_as_vector) {
    // Define the transformation matrix as per your requirement
    cv::Mat t_mat = (cv::Mat_<float>(4, 4) << 
                     -1, 0, 0, 0, 
                      0, -1, 0, 0, 
                      0, 0, 1, 0.038195,  // Adjust this to 0 if needed
                      0, 0, 0, 1);

    // Prepare an output vector to store transformed points
    std::vector<std::array<float, 3>> transformed_points;
    transformed_points.reserve(pc_as_vector.size());

    // Iterate over each point in the input vector
    for (const auto& point : pc_as_vector) {
        // Create a 4x1 homogeneous coordinate matrix for the point
        cv::Mat point_mat = (cv::Mat_<float>(4, 1) << point[0], point[1], point[2], 1.0);

        // Apply the transformation
        cv::Mat transformed_mat = t_mat * point_mat;

        // Store the transformed 3D point (ignore the homogeneous coordinate)
        transformed_points.push_back({transformed_mat.at<float>(0, 0),
                                      transformed_mat.at<float>(1, 0),
                                      transformed_mat.at<float>(2, 0)});
    }

    return transformed_points;
}




std::vector<cv::Point2i> projectLidarPoints(const std::vector<std::array<float, 3>>& lidar_points,
                                            const cv::Mat& rvec, const cv::Mat& tvec,
                                            const cv::Mat& matrix_coefficients,
                                            const cv::Mat& distortion_coefficients) {
    std::vector<cv::Point3f> points; // Temporary storage for cv::Point3f

    // Convert std::array<float, 3> to cv::Point3f
    for (const auto& point : lidar_points) {
        points.emplace_back(point[0], point[1], point[2]);
    }

    std::vector<cv::Point2f> projected_points;

    // Project the 3D lidar points to 2D image points
    cv::projectPoints(points, rvec, tvec, matrix_coefficients, distortion_coefficients, projected_points);

    // Convert the 2D points to integers
    std::vector<cv::Point2i> int_projected_points;
    for (const auto& pt : projected_points) {
        int_projected_points.push_back(cv::Point2i(static_cast<int>(pt.x), static_cast<int>(pt.y)));
    }

    return int_projected_points;
}



std::vector<cv::Point2i> undistortPoints(const std::vector<cv::Point2i>& valid_points,
                                         const cv::Mat& matrix_coefficients,
                                         const cv::Mat& distortion_coefficients) {
    // Convert valid_points to cv::Mat with type CV_32FC2 (required for undistortPoints)
    std::vector<cv::Point2f> points_float;
    for (const auto& pt : valid_points) {
        points_float.emplace_back(static_cast<float>(pt.x), static_cast<float>(pt.y));
    }
    cv::Mat points_mat(points_float);

    // Undistort points
    cv::Mat undistorted_points_mat;
    cv::undistortPoints(points_mat, undistorted_points_mat, matrix_coefficients, distortion_coefficients, cv::noArray(), matrix_coefficients);

    // Convert undistorted points back to a vector of cv::Point2i
    std::vector<cv::Point2i> undistorted_points;
    for (int i = 0; i < undistorted_points_mat.rows; ++i) {
        cv::Point2f pt = undistorted_points_mat.at<cv::Point2f>(i, 0);
        undistorted_points.emplace_back(static_cast<int>(pt.x), static_cast<int>(pt.y));
    }

    return undistorted_points;
}


void visualizeLidarPoints(const std::vector<cv::Point2i>& undistorted_points,
                          const std::vector<float>& point_intensity,
                          cv::Mat& undistorted_image) {
    // Step 1: Normalize the intensities
    float min_intensity = *std::min_element(point_intensity.begin(), point_intensity.end());
    float max_intensity = *std::max_element(point_intensity.begin(), point_intensity.end());
    
    std::vector<float> normalized_intensities;
    for (const auto& intensity : point_intensity) {
        normalized_intensities.push_back((intensity - min_intensity) / (max_intensity - min_intensity));
    }

    // Step 2: Convert normalized intensities to colormap colors
    cv::Mat intensity_mat(normalized_intensities.size(), 1, CV_32F, normalized_intensities.data());
    intensity_mat.convertTo(intensity_mat, CV_8U, 255); // Convert to range [0, 255] for colormap
    cv::Mat color_map;
    cv::applyColorMap(intensity_mat, color_map, cv::COLORMAP_RAINBOW);

    // Step 3: Draw each point on the image with the corresponding color
    for (size_t i = 0; i < undistorted_points.size(); ++i) {
        cv::Point2i point = undistorted_points[i];
        cv::Vec3b color = color_map.at<cv::Vec3b>(i, 0); // BGR color from colormap
        cv::circle(undistorted_image, point, 1, cv::Scalar(color[0], color[1], color[2]), 2);
    }
}


std::vector<bool> filterPointsWithinYaw(const std::vector<std::array<float, 3>>& lidar_points,
                                        float yaw_left, float yaw_right) {
    // Convert yaw angles from degrees to radians
    float yaw_left_rad = yaw_left * (M_PI / 180.0f);
    float yaw_right_rad = yaw_right * (M_PI / 180.0f);

    // Boolean mask to indicate if a point is within the yaw range
    std::vector<bool> mask;
    mask.reserve(lidar_points.size());

    // Compute angles and apply filter
    for (const auto& point : lidar_points) {
        float angle = std::atan2(point[1], point[0]); // Compute yaw angle of point
        mask.push_back(angle >= -yaw_right_rad && angle <= yaw_left_rad); // Apply yaw filter
    }

    return mask;
}

std::vector<std::array<float, 3>> filterLidarPoints(const std::vector<std::array<float, 3>>& lidar_points,
                                                    const std::vector<bool>& mask) {
    std::vector<std::array<float, 3>> filtered_points;
    for (size_t i = 0; i < lidar_points.size(); ++i) {
        if (mask[i]) {
            filtered_points.push_back(lidar_points[i]);
        }
    }
    return filtered_points;
}

std::vector<float> filterIntensities(const std::vector<float>& intensities,
                                     const std::vector<bool>& mask) {
    std::vector<float> filtered_intensities;
    for (size_t i = 0; i < intensities.size(); ++i) {
        if (mask[i]) {
            filtered_intensities.push_back(intensities[i]);
        }
    }
    return filtered_intensities;
}





private:
    void Callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg_image,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg_pointcloud)
    {
        
        try
        {
            
            pcl::fromROSMsg(*msg_pointcloud, pcl_cloud);
        
        // Iterate through the PCL point cloud
        for (const auto& point : pcl_cloud) {
            // Access the x, y, z, and intensity values
            // Create an array for the current point
            std::array<float, 3> point_data = {point.x, point.y, point.z};
            //float intensity = point.intensity;
            points.push_back(point_data);
            point_intensity.push_back(point.intensity);
            // Print or process the point
            //std::cout << "Point: x=" << point.x << ", y=" << point.y << ", z=" << point.z << ", intensity=" << point.intensity << std::endl;
        }  
            auto transformed_points = transformation(points);
            auto yaw_mask = filterPointsWithinYaw(transformed_points, 30.0f, 90.0f);
            std::vector<std::array<float, 3>> filtered_lidar_points = filterLidarPoints(transformed_points, yaw_mask);
            std::vector<float> filtered_intensities = filterIntensities(point_intensity, yaw_mask);
            auto projected_points = projectLidarPoints(filtered_lidar_points,rvec,tvec, matrix_coefficients_mat, distortion_coefficients_mat);
            cv::Mat image = cv_bridge::toCvShare(msg_image, "bgr8")->image;
            cv::Mat undistorted_image;
            cv::undistort(image, undistorted_image, (matrix_coefficients_mat), (distortion_coefficients_mat));

            std::vector<cv::Point2i> valid_points;
            std::vector<float> valid_intensities;
            // Get image dimensions
            int height = image.rows;
            int width = image.cols;

            // Loop over each point and check if it is within bounds
            for (size_t i = 0; i < projected_points.size(); ++i) {
                int x = projected_points[i].x;
                int y = projected_points[i].y;
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    valid_points.push_back(projected_points[i]);
                    valid_intensities.push_back(filtered_intensities[i]);
                }
            }
            auto undistort = undistortPoints(valid_points, matrix_coefficients_mat, distortion_coefficients_mat);
            
            // Convert the ROS image message to an OpenCV image

            visualizeLidarPoints(undistort,valid_intensities ,undistorted_image);

            // Display the image
            cv::namedWindow("Camera Image", cv::WINDOW_NORMAL);
            
            //cv::resize(undistorted_image, resized_image, cv::Size(), 0.5, 0.5); // Change 0.5, 0.5 to desired scale factors
            cv::imshow("Camera Image", undistorted_image);
            int key = cv::waitKey(1);
            if (key == 27) {  // 27 is the ASCII code for the Escape key
                cv::destroyWindow("Camera Image");
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
    }

    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSubscriber>());
    rclcpp::shutdown();
    return 0;
}
