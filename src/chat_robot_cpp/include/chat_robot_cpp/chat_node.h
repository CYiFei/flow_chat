#define CPPHTTPLIB_OPENSSL_SUPPORT 1
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chat_robot_cpp/msg/pcm_audio.hpp>
#include <cpp-httplib/httplib.h>
#include <nlohmann/json.hpp>
#include <iflytek_msg/msg/pcm_msg.hpp>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <map>
#include <fstream>
#include <sstream>
#include <sensor_msgs/msg/compressed_image.hpp>
// 添加action相关头文件
#include "arm_interfaces/action/arm_task.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// 添加升降服务相关头文件
#include "dual_arm_interfaces/srv/lift_controller.hpp"
// 添加base64编码支持
#include <iomanip>
#include <sstream>
#include <vector>
#include <webp/encode.h>
#include "serial_interfaces/msg/gimble_control.hpp"
#include "custom_image_msg/msg/image4m.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "app_msgs/action/go_target.hpp"
#include "app_msgs/action/semantic_navigation.hpp"
#include <tf2/LinearMath/Quaternion.h>

using StringMsg = std_msgs::msg::String;
using json = nlohmann::json;
using CompressedImage = sensor_msgs::msg::CompressedImage;

// 定义action类型别名
using ArmTask = arm_interfaces::action::ArmTask;
using ArmTaskGoalHandle = rclcpp_action::ClientGoalHandle<ArmTask>;
// 定义升降服务类型别名
using LiftController = dual_arm_interfaces::srv::LiftController;

class ChatNode;

struct StreamHandler
{
public:
    std::shared_ptr<ChatNode> node;
    std::string accumulated_response;
    std::string tts_buffer;  // 用于累积发送给TTS的文本
    bool response_start; // 标志是否是响应的开始
    std::chrono::steady_clock::time_point last_send_time; // 上次发送时间
    size_t last_check_pos; // 上次检查句子结束符的位置
    explicit StreamHandler(std::shared_ptr<ChatNode> node) : node(node), accumulated_response(""), tts_buffer(""), response_start(true) {}

    void operator()(const char *data, size_t len);
};

//MCP工具调用结果结构
struct MCPToolResult
{
    bool isError;
    std::string content;
};

//MCP工具定义结构
struct MCPTool
{
    std::string name;
    std::string description;
    json inputSchema;
    std::function<std::future<MCPToolResult>(const json& arguments)> callback;
};

// 定义SemanticItem结构体，对应Python中的dataclass
struct SemanticItem {
    float suggest_point_x = 0.0f;
    float suggest_point_y = 0.0f;
    std::string semantic_label = "";
    std::string instance_label = "";
    int orient = 0;
};

// 定义语义映射类型（按照要求修改为std::map<int, std::pair<std::string, std::string>>）
using SemanticMapping = std::map<int, std::pair<std::string, std::string>>; // index -> (chinese_name, en_name)

// 定义语义结果类型（按照要求修改为std::map<std::string, SemanticItem>）
using SemanticResult = std::map<std::string, SemanticItem>;

class ChatNode : public rclcpp::Node
{
public:
    ChatNode();

    ~ChatNode();

private:
    struct AudioInput
    {
        std::string text;
        uint8_t angle;
        rclcpp::Time timestamp;
        
        AudioInput(const std::string& t, uint8_t a, const rclcpp::Time& ts)
            : text(t), angle(a), timestamp(ts) {}
    };

    std::atomic<bool> use_audio_input_;
    rclcpp::Publisher<StringMsg>::SharedPtr tts_publisher_;
    // rclcpp::Subscription<chat_robot_cpp::msg::PcmAudio>::SharedPtr audio_sub_;
    rclcpp::Subscription<iflytek_msg::msg::PcmMsg>::SharedPtr audio_sub_;

    //用于接收停止tts信号的订阅者
    rclcpp::Subscription<StringMsg>::SharedPtr stop_tts_sub_;
    rclcpp::Publisher<StringMsg>::SharedPtr stream_tts_publisher_;
    // 输入队列
    std::queue<AudioInput> input_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    std::thread main_thread_;
    std::thread executor_thread_;  // 处理 ROS 回调

    // ASR调用，目前是云端API， 也可本地服务
    std::string simulateASR(const std::vector<float> &audio_data);

    // 模拟大模型处理
    std::string processQuery(const std::string &query);
    // 模拟大模型流式处理
    std::string processQueryStream(const std::string &query);

    // 可扩展：调用TTS播报
    void sendAudioOutput(const std::string &text);

    public:
    std::atomic<bool> stop_{false};  // 控制主循环退出
    //新增:用于控制是否停止tts发布的标志
    std::atomic<bool> stop_tts_{false};
    //新增：用于存储被中断时的部分响应
    std::string partial_response_;
    public:
    void mainLoop();
    // 提供 join 接口
    void join() {
        if (main_thread_.joinable()) {
            main_thread_.join();
        }
    }
    // 提供访问stream_tts_publisher_的方法
    rclcpp::Publisher<StringMsg>::SharedPtr getStreamTTSPublisher() {
        return stream_tts_publisher_;
    }

    //MCP相关功能
    private:
        std::vector<MCPTool> mcp_tools_;
        std::map<std::string, size_t> tool_name_to_index_;

        // ArmTask action客户端
        rclcpp_action::Client<ArmTask>::SharedPtr arm_task_client_;
        // 升降控制服务客户端
        rclcpp::Client<LiftController>::SharedPtr lift_controller_client_;

        // 添加 GoTarget action 客户端
        rclcpp_action::Client<app_msgs::action::GoTarget>::SharedPtr go_target_client_;
        // 初始化对话历史，加载角色提示
        void initializeConversationHistory();
        // 初始化外教角色提示
        void initializeExternalRolePrompt(); //初始化外教口语系统提示

        //初始化MCP工具
        void initializeMCPTools();

        //调用MCP工具
        std::future<MCPToolResult> callMCPTool(const std::string& tool_name, const json& arguments);

        //获取可用工具列表
        std::vector<MCPTool> getAvailableTools();

        //具体的MCP工具实现
        
        std::future<MCPToolResult> getWeather(const json& arguments); // 新增天气查询工具
        std::future<MCPToolResult> prepareForHomeworkTeaching(const json& arguments); // 新增作业辅导准备工具
        std::future<MCPToolResult> performHomeworkTeaching(const json& arguments); // 新增作业辅导执行工具
        std::future<MCPToolResult> endHomeworkTeaching(const json& arguments); // 新增结束作业辅导工具
        std::future<MCPToolResult> getArmImage(const json& arguments); // 获取机械臂图像
        std::future<MCPToolResult> robotWatch(const json& arguments); //获取查看的图像
        std::future<MCPToolResult> getCurrentTime(const json &arguments); //查询当前时间
        std::future<MCPToolResult> foreginTeaching(const json &arguments); //外教口语
        // 处理工具调用
        std::string processToolCall(const std::string& tool_name, const json& arguments);
        // 使用当前对话历史处理查询
        std::string processQueryWithHistory(bool has_image);
        std::string processQueryWithGpt(bool has_image);
        // 润色工具响应
        std::string polishToolResponse(const std::string& tool_name, const std::string& arguments, const std::string& raw_response);
        // ArmTask action相关函数
        std::future<MCPToolResult> sendArmTask(int32_t task_id);
        // 升降控制相关函数
        std::future<MCPToolResult> sendLiftControllerRequest(int32_t height, int32_t velocity);
        // 机器人高度调整函数
        std::future<MCPToolResult> adjustRobotHeight(const json& arguments);
        // Base64编码函数
        std::string base64_encode(const std::vector<uint8_t>& data);
        // 保存图像到文件
        void saveImageToFile(const std::vector<uint8_t>& data, const std::string& filename);
        void saveImageToFile(const std::vector<uint8_t> &data, const std::string &filename, const std::string &format);
        // 清除对话历史中的旧图像数据以提高性能
        void cleanupOldImageMessages();
        void sendGimbleControl(float hori, float vert);
    private:
        
        SemanticResult semantic_result;
        std::vector<std::pair<std::string, std::string>> furniture_list_;
        /**
         * 解析语义映射文件
         * @param file 映射文件路径
         * @return 语义映射字典
         */
        SemanticMapping parseSemanticMapping(const std::string &file);

        /**
         * 解析语义地图文件
         * @param file 语义地图文件路径
         * @param mapping_file 映射文件路径
         * @return 语义结果和映射值的pair
         */
        std::pair<SemanticResult, std::vector<std::pair<std::string, std::string>>>
        parseSemanticMap(const std::string &file, const std::string &mapping_file);

        // 获取家具列表的MCP工具函数
        std::future<MCPToolResult> getFurnitures(const json &arguments);

        // 创建GoTarget目标消息
        app_msgs::action::GoTarget::Goal createGoTargetGoal(float x, float y, float orientation);
        // app_msgs::action::GoTarget::Goal createGoTargetGoal(float x, float y, float orientation);
        // 移动到家具的实现
        std::future<MCPToolResult> moveNearFurniture(const json &arguments);

    private:
        std::vector<nlohmann::json> conversation_history_; // 对话历史
        std::vector<nlohmann::json> open_ai_conversion_history_;
        std::vector<nlohmann::json> foreign_conversion_history_;
        static constexpr int MAX_HISTORY_TURNS = 100; // 最大对话轮数

        void clearHistory(); // 清除对话历史

        // 新增：获取当前用户查询（用于保存历史对话）
        std::string current_query_;
    
    private:
        enum class agent_type {
            chat,
            foreign_teach
        };

        agent_type agent_type_ = agent_type::chat;

};

