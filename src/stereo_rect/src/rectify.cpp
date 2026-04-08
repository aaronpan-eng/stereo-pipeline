#include <chrono>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class RectifyStereoImgs : public rclcpp::Node{
    public:
        RectifyStereoImgs() : Node("rectify_stereo_imgs"), count_(0){
            // left and right cam params
            this->declare_parameter("cam0.intrinsics", std::vector<double>{0.0});
            this->declare_parameter("cam0.resolution", std::vector<int>{0});
            this->declare_parameter("cam0.distortion_coeffs", std::vector<double>{0.0});
            this->declare_parameter("cam0.distortion_model", "");
            this->declare_parameter("cam0.rostopic", "");
            this->declare_parameter("cam0.topic_type", "");

            this->declare_parameter("cam1.intrinsics", std::vector<double>{0.0});
            this->declare_parameter("cam1.resolution", std::vector<double>{0});
            this->declare_parameter("cam1.distortion_coeffs", std::vector<double>{0.0});
            this->declare_parameter("cam1.distortion_model", "");
            this->declare_parameter("cam1.T_cn_cnm1", std::vector<double>{0.0});      // 4x4 extrinsic matrix
            this->declare_parameter("cam1.rostopic", "");
            this->declare_parameter("cam1.topic_type", "");

            // extract values from params
            auto cam0_intrinsics_ = this->get_parameter("cam0.intrinsics").as_double_array();
            auto cam0_resolution_ = this->get_parameter("cam0.resolution").as_integer_array();
            auto cam0_dist_coeffs_ = this->get_parameter("cam0.distortion_coeffs").as_double_array();
            auto cam0_dist_model_ = this->get_parameter("cam0.distortion_model").as_string();
            auto cam0_rostopic_ = this->get_parameter("cam0.rostopic").as_string();
            auto cam0_topic_type_ = this->get_parameter("cam0.topic_type").as_string();

            auto cam1_intrinsics_ = this->get_parameter("cam1.intrinsics").as_double_array();
            auto cam1_resolution_ = this->get_parameter("cam1.resolution").as_integer_array();
            auto cam1_dist_coeffs_ = this->get_parameter("cam1.distortion_coeffs").as_double_array();
            auto cam1_dist_model_ = this->get_parameter("cam1.distortion_model").as_string();
            auto cam1_rostopic_ = this->get_parameter("cam1.rostopic").as_string();
            auto cam1_topic_type_ = this->get_parameter("cam1.topic_type").as_string();
            auto cam1_T_cn_cnm1_ = this->get_parameter("cam1.T_cn_cnm1").as_double_array();

            // set subscribers based on topic type
            if (cam0_topic_type_ == "Image" && cam1_topic_type_ == "Image"){
                sub_left_.emplace();
                sub_right_.emplace();
                sub_left_->subscribe(this, cam0_rostopic_);
                sub_right_->subscribe(this, cam1_rostopic_);

                // sync_.emplace();
                sync_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>(
                    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>(10), *sub_left_, *sub_right_);
                sync_.value()->registerCallback(std::bind(&RectifyStereoImgs::rectify, this, std::placeholders::_1, std::placeholders::_2));
            }else if(cam0_topic_type_ == "CompressedImage" && cam1_topic_type_ == "CompressedImage"){
                sub_left_compressed_.emplace();
                sub_right_compressed_.emplace();
                sub_left_compressed_->subscribe(this, cam0_rostopic_);
                sub_right_compressed_->subscribe(this, cam1_rostopic_);

                // sync_compressed_.emplace();
                sync_compressed_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>>>(
                    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>(10), *sub_left_compressed_, *sub_right_compressed_);
                sync_compressed_.value()->registerCallback(std::bind(&RectifyStereoImgs::rectify_comp, this, std::placeholders::_1, std::placeholders::_2));
            }else{
                throw std::runtime_error("Unsupported topic_type cam0: " + cam0_topic_type_ + " \n   and cam1: " + cam1_topic_type_ + "\n");
            }
            

            // set publishers
            pub_left_rect_ = this->create_publisher<sensor_msgs::msg::Image>("/cam_sync/cam0/image_rect", 10);
            pub_right_rect_ = this->create_publisher<sensor_msgs::msg::Image>("/cam_sync/cam1/image_rect", 10);
            pub_left_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/cam_sync/cam0/rect_info", 10);
            pub_right_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/cam_sync/cam1/rect_info", 10);

            // for left and right calibration intrinsics
            const auto fx1 = cam0_intrinsics_[0];
            const auto fy1 = cam0_intrinsics_[1];
            const auto cx1 = cam0_intrinsics_[2];
            const auto cy1 = cam0_intrinsics_[3];

            cv::Mat K0_ = (cv::Mat_<double>(3,3) << fx1, 0.0, cx1,
                0.0, fy1, cy1,
                0.0, 0.0, 1.0);

            cv::Mat D0 = (cv::Mat_<double>(1,4) << cam0_dist_coeffs_[0], cam0_dist_coeffs_[1], cam0_dist_coeffs_[2], cam0_dist_coeffs_[3]);

            const auto fx2 = cam1_intrinsics_[0];
            const auto fy2 = cam1_intrinsics_[1];
            const auto cx2 = cam1_intrinsics_[2];
            const auto cy2 = cam1_intrinsics_[3];

            cv::Mat K1_ = (cv::Mat_<double>(3,3) << fx2, 0.0, cx2,
                0.0, fy2, cy2,
                0.0, 0.0, 1.0);

            cv::Mat D1 = (cv::Mat_<double>(1,4) << cam1_dist_coeffs_[0], cam1_dist_coeffs_[1], cam1_dist_coeffs_[2], cam1_dist_coeffs_[3]);
            // cv::Mat T_cn_cm1 = (cv::Mat_<double> << )
        }

    private:
        size_t count_;
        // publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_rect_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_rect_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_info_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_info_;

        // below is set based on topic type for subscriber
        // optional subscribers for Image
        std::optional<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_left_;
        std::optional<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_right_;
        std::optional<std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>>>> sync_;
        
        // optional subscribers for CompressedImage
        std::optional<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> sub_left_compressed_;
        std::optional<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> sub_right_compressed_;
        std::optional<std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>>>> sync_compressed_;
        
        void rectify(const sensor_msgs::msg::Image::ConstSharedPtr &left, const sensor_msgs::msg::Image::ConstSharedPtr &right){}

        void rectify_comp(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &left, const sensor_msgs::msg::CompressedImage::ConstSharedPtr &right){}
};
    

