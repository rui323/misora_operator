#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "misora_gui/misora_gui_sub_component.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;
    auto node = std::make_shared<component_operator_gui_sub::DistributeImage_sub>();
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
}