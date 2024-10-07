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
            
            // Convert the ROS image message to an OpenCV image
            cv::Mat image = cv_bridge::toCvShare(msg_image, "bgr8")->image;
            cv::Mat undistorted_image;
            cv::undistort(image, undistorted_image, (matrix_coefficients_mat), (distortion_coefficients_mat));




            // Display the image
            cv::namedWindow("Camera Image", cv::WINDOW_NORMAL);
            
            cv::resize(undistorted_image, resized_image, cv::Size(), 0.5, 0.5); // Change 0.5, 0.5 to desired scale factors
            cv::imshow("Camera Image", resized_image);
            cv::waitKey(1); 
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
