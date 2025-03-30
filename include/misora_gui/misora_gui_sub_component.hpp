#ifndef MISORA_GUI_SUB_COMPONENT_HPP
#define MISORA_GUI_SUB_COMPONENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;

#include "misora_gui/cv_mat_type_adapter.hpp"

namespace component_operator_gui_sub
{
class DistributeImage_sub : public rclcpp::Node
{
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;
    public:
    std::string result_data, id;
    cv::Mat result_image;

    explicit DistributeImage_sub(const rclcpp::NodeOptions &options);
    DistributeImage_sub() : DistributeImage_sub(rclcpp::NodeOptions{}) {}

    private:
    std::map<std::string, rclcpp::Subscription<MyAdaptedType>::SharedPtr> image_subscribers_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> string_subscribers_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr category_publisher_;
};
} // namespace component_operator_gui_sub

#endif // MISORA_GUI_SUB_COMPONENT_HPP