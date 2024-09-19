#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class CameraSubscriber : public rclcpp::Node
{
        //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pointcloud_sub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> MySyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
public:
    CameraSubscriber()
        : Node("camera_subscriber")
    {

        YAML::Node calib_data = YAML::LoadFile("/home/abd1340m/Dokumente/c++/ros2_cpp_vis_ws/src/visualise_cpp/src/front_mid_teleop.yaml");
        std::vector<double> matrix_coefficients = calib_data["camera_matrix"]["data"].as<std::vector<double>>();
        std::vector<double> distortion_coefficients = calib_data["distortion_coefficients"]["data"].as<std::vector<double>>();

        // Convert to OpenCV Mat
        cv::Mat matrix_coefficients_mat(3, 3, CV_64F, matrix_coefficients.data());
        cv::Mat distortion_coefficients_mat(distortion_coefficients.size(), 1, CV_64F, distortion_coefficients.data());
        // Replace "/camera/image" with the correct topic name for your use case
        //image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        //    "/my_camera/pylon_ros2_camera_node/image_raw", 10, std::bind(&CameraSubscriber::imageCallback, this, std::placeholders::_1));

         // Create subscribers for Image and PointCloud2
        //image_sub_.subscribe(this, "/image_raw", rmw_qos_profile_sensor_data);
        //pointcloud_sub_.subscribe(this, "/lidar/points", rmw_qos_profile_sensor_data);
        image_sub_.subscribe(this, "/my_camera/pylon_ros2_camera_node/image_raw", rmw_qos_profile_sensor_data);
        pointcloud_sub_.subscribe(this, "/ouster/points", rmw_qos_profile_sensor_data);


        auto sync_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>(10);
        sync_policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.3)); // Set the slop to 0.05 seconds
        sync_policy.setInterMessageLowerBound(0, rclcpp::Duration(std::chrono::milliseconds(100)));
        
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
            // Convert the ROS image message to an OpenCV image
            cv::Mat image = cv_bridge::toCvShare(msg_image, "bgr8")->image;

            // Display the image
            //cv::namedWindow("Camera Image", cv::WINDOW_NORMAL);
             cv::Mat resized_image;
            cv::resize(image, resized_image, cv::Size(), 0.5, 0.5); // Change 0.5, 0.5 to desired scale factors
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
