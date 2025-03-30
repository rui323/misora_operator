#ifndef MISORA_GUI_COMPONENT_HPP
#define MISORA_GUI_COMPONENT_HPP

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <algorithm>

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/type_adapter.hpp>

#include "misora_gui/gui_tool.hpp"
#include "misora_gui/cv_mat_type_adapter.hpp"

using namespace std::chrono_literals;

namespace component_operator_gui
{
class DistributeImage : public rclcpp::Node
{
public:
    using MyAdaptedType = rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>;

    cv::Mat mat;
    std::vector<std::string> buttons_name;//表示しているボタンのリスト
    std::vector<Button> buttons_; // ボタン位置、サイズのリスト
    cv::Size btn_size;

    explicit DistributeImage(const rclcpp::NodeOptions &options);
    DistributeImage() : DistributeImage(rclcpp::NodeOptions{}) {}

    cv::Mat setup(std::string n,std::vector<std::string> p);

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    void timer_callback();
    void mouse_click_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void rewriteImage(cv::Point sp, cv::Point ep, std::string text, int btn_W, int btn_H, cv::Scalar color) const;

    rclcpp::Publisher<MyAdaptedType>::SharedPtr publish_gui;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr click_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr return_name_;
    rclcpp::TimerBase::SharedPtr view_on;

    std::map<std::string, rclcpp::Time> button_press_times_;
    
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> bool_publishers_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> string_publishers_;
    std::map<std::string, rclcpp::Publisher<MyAdaptedType>::SharedPtr> image_publishers_;
};

} // namespace component_operator_gui

#endif // MISORA_GUI_COMPONENT_HPP