#include "stereo_rect/rectify.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RectifyStereoImgs>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}