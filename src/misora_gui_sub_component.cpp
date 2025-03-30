#include "misora_gui/misora_gui_sub_component.hpp"

namespace component_operator_gui_sub
{
DistributeImage_sub::DistributeImage_sub(const rclcpp::NodeOptions &options)
    : Node("distribute_image_sub",options)
{
    // P1,P2,P3,P4,P6モード
    this->declare_parameter("sub_parameter", "P6");
    std::string param = this->get_parameter("sub_parameter").as_string();

    std::map<std::string, std::vector<std::string>> sub_topics = {
        {"P1", {"meter", "qr", "0 | 1"}},
        {"P2", {"meter", "qr", "OP | CL", "0 | 1"}},
        {"P3", {"crack", "qr", "thick"}},
        {"P4", {"qr", "disaster", "0 | 1", "debris"}},
        {"P6",{"meter", "qr", "debris", "disaster", "missing"}},
    };
    
    category_publisher_ = this->create_publisher<std_msgs::msg::String>("return_name", 10);

    // 1. 指定されたパラメータに対応するトピック一覧を取得
    if (sub_topics.find(param) != sub_topics.end()) {
        for (const auto &topic : sub_topics[param]) {
            // 2. 各トピックごとに subscriber を作成
            if(topic == "meter" || topic == "qr" || topic == "crack" || topic == "thick"){
                string_subscribers_[topic] = this->create_subscription<std_msgs::msg::String>(
                    topic+"_result_data",10,
                    [this, topic](const std_msgs::msg::String::SharedPtr msg){
                        if(topic == "qr") id = msg->data;
                        else result_data = msg->data;
                        // RCLCPP_INFO_STREAM(this->get_logger(),"Recieved message: " << msg->data);
                        std_msgs::msg::String return_name;
                        return_name.data = topic;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        this->category_publisher_->publish(return_name);
                    }
                );
                image_subscribers_[topic] = this->create_subscription<MyAdaptedType>(
                    topic+"_result_image",10,
                    [this, topic](const std::unique_ptr<cv::Mat> msg){
                        if(!(topic == "qr")) result_image = std::move(*msg);
                    }
                );
            }
            else {
                std::string topic_name = topic;
                if(topic == "0 | 1") topic_name = "bulb_operate";
                else if(topic == "OP | CL") topic_name = "bulb";

                string_subscribers_[topic] = this->create_subscription<std_msgs::msg::String>(
                    topic_name+"_report",10,
                    [this, topic](const std_msgs::msg::String::SharedPtr msg){
                        result_data = msg->data;
                        // RCLCPP_INFO_STREAM(this->get_logger(),"Recieved message: " << msg->data);
                        std_msgs::msg::String return_name;
                        return_name.data = topic;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        this->category_publisher_->publish(return_name);
                    }
                );
                image_subscribers_[topic] = this->create_subscription<MyAdaptedType>(
                    topic_name+"_image",10,
                    [this, topic](const std::unique_ptr<cv::Mat> msg){
                        RCLCPP_INFO_STREAM(this->get_logger(),"Recieved message address: "<< &(msg->data));
                        result_image = msg->clone();
                    }
                );
            }
            
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid profile: %s", param.c_str());
    }
    
}

}//namespace component_oprator_gui_sub


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_operator_gui_sub::DistributeImage_sub)