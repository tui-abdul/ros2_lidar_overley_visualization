#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraSubscriber : public rclcpp::Node
{
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
public:
    CameraSubscriber()
        : Node("camera_subscriber")
    {
        // Replace "/camera/image" with the correct topic name for your use case
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/my_camera/pylon_ros2_camera_node/image_raw", 10, std::bind(&CameraSubscriber::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Convert the ROS image message to an OpenCV image
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

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
