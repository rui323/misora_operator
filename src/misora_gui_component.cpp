#include "misora_gui/misora_gui_component.hpp"

namespace component_operator_gui
{
DistributeImage::DistributeImage(const rclcpp::NodeOptions &options) 
    : Node("distribute_image", options)
{
    this->declare_parameter("my_parameter", "P6");
    std::string param = this->get_parameter("my_parameter").as_string();
    RCLCPP_INFO(this->get_logger(), "Received my_parameter: %s", param.c_str());

//クフウシヤ提案ボタン表示名
    std::map<std::string, std::vector<std::string>> pub_topics = {
        {"P1", {"pressure", "qr", "V_maneuve", "send"}},
        {"P2", {"pressure", "qr", "V_maneuve", "V_state", "send"}},
        {"P3", {"cracks", "qr", "metal_loss", "send"}},
        {"P4", {"qr", "disaster", "debris", "V_maneuve", "send"}},
        {"P6", {"pressure", "qr", "debris", "disaster", "missing", "send"}}
    };

    if (pub_topics.find(param) != pub_topics.end()) {
        for (const auto &topic : pub_topics[param]) { //degital twin がまだ決まってない　meter,qr,crack,thickness(thick)はbool その他はstringとImageを用意しとく
            if(topic == "pressure" || topic == "qr" || topic == "cracks" || topic == "metal_loss" || topic == "send"){
                bool_publishers_[topic] = (this->create_publisher<std_msgs::msg::Bool>(topic+"_trigger", 10));
            }
            else{
                std::string topic_name = topic;
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

    std::map<std::string, bool> received_initialize;
    for(const auto& key : pub_topics[param]) received_initialize[key] = false;
    receive_list.push_back(received_initialize);

    mat = setup(pub_topics[param]);

    view_on = this->create_wall_timer(500ms, std::bind(&DistributeImage::timer_callback, this));
}



void DistributeImage::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    auto it = std::find(buttons_name.begin(),buttons_name.end(),msg->data);
    int index = std::distance(buttons_name.begin(), it);

    for (auto& pair_map : receive_list) {
        for (auto& pair : pair_map){
            if (pair.first != "qr") {
                pair.second = false;
            }
        }
    }
    receive_list.push_back(std::map<std::string, bool>{{msg->data, true}});

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
            
            if(button_name == "pressure" || button_name == "qr" || button_name == "cracks" || button_name == "metal_loss" || button_name == "send"){ //被災者の顔写真を送るのか、QRのデコードならいらないかも
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
                
                if(button_name == "V_state" && bulb_state_count == 0){
                    msg_s.data = "OPEN";
                    bulb_state_count = 1;
                }
                else if(button_name == "V_state" && bulb_state_count == 1){
                    msg_s.data = "CLOSE";
                    bulb_state_count = 0;
                }

                if(button_name == "V_maneuve") msg_s.data = "1";//完了の信号
                
                string_publishers_[button_name]->publish(msg_s);
            }
            
            
            RCLCPP_INFO(this->get_logger(), "Button '%s' clicked",button_name.c_str());
        }
    }
}

void DistributeImage::rewriteImage(cv::Point sp, cv::Point ep, std::string text, int btn_W, int btn_H, cv::Scalar color) const {
    cv::rectangle(mat, sp, ep, color, cv::FILLED);
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, 2, &baseline);
    cv::Point tp(sp.x + (btn_W - text_size.width) / 2, sp.y + (btn_H + text_size.height) / 2);
    cv::putText(mat, text, tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, cv::Scalar(0, 0, 0), 2);
    
    std::string qr_text,other_text;
    for (auto& item_map : receive_list) {  
        for (auto& item : item_map) {  // item は std::pair<const std::string, bool>
            if (item.second) {  
                if (item.first == "qr") { 
                    qr_text = "qr, ";
                } else {
                    other_text = item.first;
                }
            }
        }
    }
    std::string t = "Receive: " + qr_text + other_text; 
    cv::Rect button_rect(buttons_.back().pos, buttons_.back().size);
    cv::Point receive_btn_sp = cv::Point(button_rect.x, button_rect.y);
    cv::Point receive_btn_ep = cv::Point(receive_btn_sp.x+buttons_.back().size.width,receive_btn_sp.y+buttons_.back().size.height);

    cv::rectangle(mat, receive_btn_sp, receive_btn_ep, 0, cv::FILLED);
    baseline = 0;
    text_size = cv::getTextSize(t, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, 2, &baseline);
    tp = cv::Point(buttons_.back().pos.x + (buttons_.back().size.width - text_size.width) / 2, buttons_.back().pos.y+ (buttons_.back().size.height + text_size.height) / 2);
    cv::putText(mat, t, tp, cv::FONT_HERSHEY_SIMPLEX|cv::FONT_ITALIC, 0.78, cv::Scalar(255, 255, 255), 2);
    
}

cv::Mat DistributeImage::setup(std::vector<std::string> p){
    int width = 300, height = 350;
    DrawTool canvas(width,height,0);//画面サイズ
    
    int btn_width = 140, btn_height = 50;//ボタンのサイズ
    btn_size = cv::Size(btn_width,btn_height);
    int x_offset = 5,y_offset = 25;//ボタンの最初の位置
    int btn_per_row = 2;//一行に何個ボタン設置するか
    int btn_space_row = 5;//ボタン間のスペース
    int btn_space_col = 30;

    for(size_t i = 0; i < p.size(); i++){
        int row = i / btn_per_row;  // 行数
        int col = i % btn_per_row;  // 列数

        // ボタン位置を更新
        Button btn(cv::Point(x_offset + col * (btn_width + btn_space_row), y_offset + row * (btn_height + btn_space_col)),cv::Size(btn_width,btn_height));
        buttons_.push_back(btn); // ボタンをリストに追加
        canvas.drawButton(btn, p[i], cv::Scalar(255, 255, 255), -1, cv::LINE_AA, 0.78, cv::Scalar(0,0,0), 2);
    }

    Button btn(cv::Point(x_offset,buttons_.back().pos.y+btn_height+15),cv::Size(btn_width*2+btn_space_row,btn_height+30));
    buttons_.push_back(btn);
    canvas.drawButton(buttons_.back(), "Receive: None", cv::Scalar(0, 0, 0), -1, cv::LINE_AA, 0.78, cv::Scalar(255,255,255), 2);

    return canvas.getImage();
}

} // namespace component_operator_gui

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(component_operator_gui::DistributeImage)