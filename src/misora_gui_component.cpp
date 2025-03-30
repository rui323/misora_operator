#include "misora_gui/misora_gui_component.hpp"

namespace component_operator_gui
{
DistributeImage::DistributeImage(const rclcpp::NodeOptions &options) 
    : Node("distribute_image", options)
{
    this->declare_parameter("my_parameter", "P6");
    std::string param = this->get_parameter("my_parameter").as_string();
    RCLCPP_INFO(this->get_logger(), "Received my_parameter: %s", param.c_str());

    std::map<std::string, std::vector<std::string>> pub_topics = {
        {"P1", {"meter", "qr", "0 | 1", "send"}},
        {"P2", {"meter", "qr", "0 | 1", "OP | CL", "send"}},
        {"P3", {"crack", "qr", "thick", "send"}},
        {"P4", {"qr", "disaster", "debris", "0 | 1", "send"}},
        {"P6", {"meter", "qr", "debris", "disaster", "missing", "send"}}
    };

    if (pub_topics.find(param) != pub_topics.end()) {
        for (const auto &topic : pub_topics[param]) { //degital twin がまだ決まってない　meter,qr,crack,thickness(thick)はbool その他はstringとImageを用意しとく
            if(topic == "meter" || topic == "qr" || topic == "crack" || topic == "thick" || topic == "send"){
                bool_publishers_[topic] = (this->create_publisher<std_msgs::msg::Bool>(topic+"_trigger", 10));
            }
            else{
                std::string topic_name = topic;
                if(topic == "0 | 1") topic_name = "bulb_operate";
                if(topic == "OP | CL") topic_name = "bulb";
                string_publishers_[topic] = (this->create_publisher<std_msgs::msg::String>(topic_name+"_report", 10));
                image_publishers_[topic] = (this->create_publisher<MyAdaptedType>(topic_name+"_image", 10));
            }
            RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s", topic.c_str());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid profile: %s", param.c_str());
    }

    buttons_name = pub_topics[param];

    publish_gui = this->create_publisher<MyAdaptedType>("gui_with_buttons",1);
    click_ = this->create_subscription<geometry_msgs::msg::Point>(
        "gui_with_buttons_mouse_left", 10, std::bind(&DistributeImage::mouse_click_callback, this, std::placeholders::_1));

    return_name_ = this->create_subscription<std_msgs::msg::String>(
        "return_name", 10, std::bind(&DistributeImage::topic_callback, this, std::placeholders::_1));

    mat = setup(param,pub_topics[param]);

    view_on = this->create_wall_timer(500ms, std::bind(&DistributeImage::timer_callback, this));
}



void DistributeImage::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    auto it = std::find(buttons_name.begin(),buttons_name.end(),msg->data);
    int index = std::distance(buttons_name.begin(), it);

    //時間計測
    rclcpp::Time press_time = button_press_times_[msg->data];
    rclcpp::Time receive_time = this->now();
    double elapsed_time = (receive_time - press_time).seconds(); // 経過時間を計算
    RCLCPP_INFO(this->get_logger(), "Response received for '%s': Time elapsed = %.3f seconds", 
                msg->data.c_str(), elapsed_time);
    //

    cv::Point start_pos = buttons_[index].pos;
    cv::Point end_pos = cv::Point(buttons_[index].pos.x + btn_size.width, buttons_[index].pos.y + btn_size.height);
    rewriteImage(start_pos,end_pos,msg->data,btn_size.width,btn_size.height,cv::Scalar(255,255,255));
    publish_gui->publish(mat);
}

void DistributeImage::timer_callback() {
    publish_gui->publish(mat);
}

void DistributeImage::mouse_click_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    cv::Point point(msg->x, msg->y);
    for(size_t i = 0; i < buttons_.size(); i++){
        
        // ボタンの矩形領域を定義
        cv::Rect button_rect(buttons_[i].pos, buttons_[i].size);
        // クリック位置がボタンの範囲内にあるかチェック
        if (button_rect.contains(point)) {
            cv::Point sp = cv::Point(button_rect.x, button_rect.y);
            cv::Point ep = cv::Point(button_rect.x + btn_size.width, button_rect.y + btn_size.height);
            std::string button_name = buttons_name[i];
            button_press_times_[button_name] = this->now(); // 押下時刻を保存

            rewriteImage(sp,ep,button_name,btn_size.width,btn_size.height,cv::Scalar(0,0,255));
            // クリックされたボタンを赤色にした状態でGUIを再描画
            publish_gui->publish(mat);
            
            if(button_name == "meter" || button_name == "qr" || button_name == "crack" || button_name == "thick" || button_name == "send"){ //被災者の顔写真を送るのか、QRのデコードならいらないかも
                std_msgs::msg::Bool msg_b;
                msg_b.data = true;
                bool_publishers_[button_name]->publish(msg_b);
            }
            else { //MISORA PCから送られてきた画像をそのまま流す
                
                cv::Scalar color;
                // 空画像を生成　本当はmisoraから未加工画像をもらっている
                color = cv::Scalar(255,0,0);
               
                DrawTool null_image(1000,1000,color);
                std::unique_ptr<cv::Mat> msg_i = std::make_unique<cv::Mat>(null_image.getImage());
                RCLCPP_INFO_STREAM(this->get_logger(), "address "<<&(msg_i->data));
                image_publishers_[button_name]->publish(std::move(msg_i));

                std_msgs::msg::String msg_s;
                msg_s.data = "OK";
                string_publishers_[button_name]->publish(msg_s);
            }
            //半分領域設定まだ
            // else if(button_name == "0 | 1"){ //ボタンの領域を半分にして 作業完了：１、未完了：０
            //     std_msgs::msg::String msg_complete;
            //     msg_complete.data = "1";
            //     string_publishers_["bulb"]->publish(msg_complete);
            // }
            // else if(button_name == "OP | CL"){ //ボタンの領域を半分にしてOPEN / CLOSEどっちでもおくれるように
            //     std_msgs::msg::String msg_OorC;
            //     msg_OorC.data = "OPEN";
            //     string_publishers_["bulb_report"]->publish(msg_OorC);
            // }
            
            RCLCPP_INFO(this->get_logger(), "Button '%s' clicked",button_name.c_str());
        }
    }
}

void DistributeImage::rewriteImage(cv::Point sp, cv::Point ep, std::string text, int btn_W, int btn_H, cv::Scalar color) const {
    cv::rectangle(mat, sp, ep, color, cv::FILLED);
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, &baseline);
    cv::Point tp(sp.x + (btn_W - text_size.width) / 2, sp.y + (btn_H + text_size.height) / 2);
    cv::putText(mat, text, tp, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
}

cv::Mat DistributeImage::setup(std::string n,std::vector<std::string> p){
    int width = 300, height = 350;
    if (n == "P1" || n == "P3") {
        height = 250;
    }
    DrawTool canvas(width,height,0);//画面サイズ
    
    int btn_width = 120, btn_height = 50;//ボタンのサイズ
    btn_size = cv::Size(btn_width,btn_height);
    int x_offset = 20,y_offset = 50;//ボタンの最初の位置
    int btn_per_row = 2;//一行に何個ボタン設置するか
    int btn_space_row = 20;//ボタン間のスペース
    int btn_space_col = 50;

    for(size_t i = 0; i < p.size(); i++){
        int row = i / btn_per_row;  // 行数
        int col = i % btn_per_row;  // 列数

        // ボタン位置を更新
        Button btn(cv::Point(x_offset + col * (btn_width + btn_space_row), y_offset + row * (btn_height + btn_space_col)),cv::Size(btn_width,btn_height));
        buttons_.push_back(btn); // ボタンをリストに追加
        canvas.drawButton(btn, p[i], cv::Scalar(255, 255, 255), -1, cv::LINE_AA, 1, 0, 2);
    }
    return canvas.getImage();
}

} // namespace component_operator_gui

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_operator_gui::DistributeImage)