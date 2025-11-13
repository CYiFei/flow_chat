// stream_subscriber_demo.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using StringMsg = std_msgs::msg::String;

class StreamTtsSubscriber : public rclcpp::Node
{
public:
    StreamTtsSubscriber() : Node("stream_tts_subscriber")
    {
        // 创建订阅者
        subscription_ = this->create_subscription<StringMsg>(
            "/personate_tts_text",
            10,  // 队列大小
            [this](const StringMsg::SharedPtr msg) {
                this->onMessageReceived(msg);
            });

        RCLCPP_INFO(this->get_logger(), "✅ Listening to /interaction_tts_text for streaming text...");
        RCLCPP_INFO(this->get_logger(), "💡 Tip: Send '[DONE]' to end the current stream.");
    }

private:
    void onMessageReceived(const StringMsg::SharedPtr msg)
    {
        std::string text = msg->data;

        // 过滤空字符串
        if (text.empty()) {
            return;
        }
        std::cout << "temp text = " << text << std::flush;
        std::cout << std::endl;
        // 检查是否是结束标记
        if (text == "[DONE]" || text == "[END]") {
            finalizeStream();
            return;
        }

        // 累加文本
        accumulated_text_ += text;

        // 实时显示当前 token（可选）
        RCLCPP_DEBUG(this->get_logger(), "Token: '%s'", text.c_str());
    }

    void finalizeStream()
    {
        if (!accumulated_text_.empty()) {
            RCLCPP_INFO(this->get_logger(), 
                "🎉 Stream completed. Full text:\n---\n%s\n---", 
                accumulated_text_.c_str());
            accumulated_text_.clear();
        } else {
            RCLCPP_WARN(this->get_logger(), "Stream ended, but no content was received.");
        }
    }

    rclcpp::Subscription<StringMsg>::SharedPtr subscription_;
    std::string accumulated_text_;  // 存储拼接的完整文本
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StreamTtsSubscriber>();

    // 启动自旋，处理回调
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}