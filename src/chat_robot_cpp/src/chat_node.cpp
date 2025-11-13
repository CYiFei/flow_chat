#include <chat_robot_cpp/chat_node.h>

ChatNode::ChatNode() : Node("chat_node")
{
    // 参数：是否使用音频输入
    bool temp = false;
    this->declare_parameter("use_audio_input", false);
    this->get_parameter("use_audio_input", temp);
    use_audio_input_.store(temp);
    // 创建发布者（用于输出tts文本）
    // tts_publisher_ = this->create_publisher<StringMsg>("/tts_text", 10);

    // 创建流式TTS发布者（用于发布到/interaction_tts_text）
    stream_tts_publisher_ = this->create_publisher<StringMsg>("/personate_tts_text", 10);

    // 创建升降控制服务客户端
    lift_controller_client_ = this->create_client<dual_arm_interfaces::srv::LiftController>("lift_controller");
    // 创建机械臂任务action客户端
    arm_task_client_ = rclcpp_action::create_client<ArmTask>(this, "/action_manager");

    // 在构造函数中添加
    go_target_client_ = rclcpp_action::create_client<app_msgs::action::GoTarget>(this, "/go_target_action");
    // 订阅 /pcm_tf 主题, 使用 iflytek::msg::PcmMsg
    audio_sub_ = this->create_subscription<iflytek_msg::msg::PcmMsg>(
        "/pcm_tf",
        10,
        [this](const iflytek_msg::msg::PcmMsg::SharedPtr msg)
        {
            // ✅ 校验 length 和 pcm_buf 长度是否一致（可选）
            if (static_cast<int32_t>(msg->pcm_buf.size()) != msg->length)
            {
                RCLCPP_WARN(this->get_logger(),
                            "Received PcmMsg with length %d but pcm_buf size is %zu",
                            msg->length, msg->pcm_buf.size());
            }

            // ✅ 提取文本
            std::string asr_text = msg->pcm_buf;

            // 如果文本为空，跳过
            if (asr_text.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Received empty pcm_buf from /pcm_tf");
                return;
            }

            // ✅ 加锁并推入队列
            std::lock_guard<std::mutex> lock(queue_mutex_);
            input_queue_.push({asr_text,
                               msg->angle, // 直接使用 uint8
                               this->now()});
            queue_cv_.notify_one();
        });

    // 新增：订阅停止TTS信号主题
    stop_tts_sub_ = this->create_subscription<StringMsg>(
        "/stop_tts",
        10,
        [this](const StringMsg::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received stop TTS signal: %s", msg->data.c_str());
            // 设置停止TTS标志
            stop_tts_.store(true);
        });

    // 先尝试从安装目录查找，再尝试从源码目录查找
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("chat_robot_cpp");
    std::string semantic_map_path = package_share_directory + "/data/semantic_map.data";
    std::string semantic_dict_path = package_share_directory + "/data/semantic_dictionary.txt";

    // 如果安装目录找不到，则尝试源码目录
    if (!std::filesystem::exists(semantic_map_path) || !std::filesystem::exists(semantic_dict_path))
    {
        // 尝试相对路径（相对于可执行文件）
        semantic_map_path = "data/semantic_map.data";
        semantic_dict_path = "data/semantic_dictionary.txt";

        // 或者使用构建时的源码路径
        std::string source_data_path = "../src/chat_robot_cpp/data";
        if (std::filesystem::exists(source_data_path + "/semantic_map.data"))
        {
            semantic_map_path = source_data_path + "/semantic_map.data";
            semantic_dict_path = source_data_path + "/semantic_dictionary.txt";
        }
    }

    // parseSemanticMap(semantic_map_path, semantic_dict_path);
    auto semantic_data = parseSemanticMap(semantic_map_path, semantic_dict_path);
    semantic_result = semantic_data.first;
    furniture_list_ = semantic_data.second;
    // 初始化对话历史，加载角色提示
    initializeConversationHistory();
    // 初始化MCP工具
    initializeMCPTools();

    // 启动主循环线程
    // main_thread_ = std::thread(&ChatNode::mainLoop, this);
    RCLCPP_INFO(this->get_logger(), "ChatNode started. Subscribed to /pcm_tf");
}

ChatNode::~ChatNode()
{
    if (main_thread_.joinable())
    {
        main_thread_.join();
    }
}

void ChatNode::mainLoop()
{
    // 创建 executor 处理 ROS 回调（如订阅者）
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(shared_from_this());

    // ✅ 使用成员变量，而不是局部变量
    executor_thread_ = std::thread([this, &executor]()
                                   {
        while (rclcpp::ok() && !stop_.load()) {
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_INFO(this->get_logger(), "Executor thread exited."); });

    RCLCPP_INFO(this->get_logger(), "ChatNode main loop started. Type 'quit' to exit.");

    while (rclcpp::ok() && !stop_.load())
    {
        std::string query;
        if (!use_audio_input_)
        {
            // 文本模式：从终端输入
            std::cout << "\nQuery (Type 'quit' to exit): ";
            std::getline(std::cin, query);
            if (query.empty())
                continue;
            if (query == "quit")
            {
                stop_.store(true);
                break;
            }
            // 只在文本模式下加入了清除历史对话的内容，话题订阅那块还没有加清除历史对话功能
            if (query == "clear")
            {
                clearHistory();
                std::cout << "[System] 对话历史已清空。" << std::endl;
                continue;
            }
        }
        else
        {
            // 语音模式：从队列获取识别结果
            std::unique_lock<std::mutex> lock(queue_mutex_);
            // queue_cv_.wait(lock, [this]
            //                { return !input_queue_.empty(); });
            // ✅ 添加超时或退出条件
            queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]
                               { return !input_queue_.empty() || !rclcpp::ok() || stop_.load(); });

            if (!rclcpp::ok() || stop_.load())
            {
                break;
            }

            if (input_queue_.empty())
                continue;

            auto audio_input = input_queue_.front();
            input_queue_.pop();
            lock.unlock();
            // std::string asr_result = simulateASR(audio_input.data);
            if (audio_input.text.empty())
            {
                RCLCPP_WARN(this->get_logger(), "ASR returned empty result.");
                continue;
            }
            query = audio_input.text;
        }

        // 保存当前查询，用于可能的历史记录保存
        current_query_ = query;

        // 处理查询（模拟大模型)
        // std::string response = processQuery(query);
        std::string response = processQueryStream(query);

        // 如果响应标记为已发送TTS，则跳过重复发送
        if (response == "[TTS_MESSAGE_SENT]") {
            continue;
        }

        // 检查是否被中断
        if (stop_tts_.load())
        {
            // 重置停止标志
            stop_tts_.store(false);
            // 继续下一个循环等待新的ASR输入
            continue;
        }

        std::string history = "[User] " + query + "\n[AI] " + response;

        // 打印输出
        std::cout << "\n[ALL Output]\n"
                  << history << std::endl;

        // 如果是语音模式，发送TTS消息
        if (use_audio_input_)
        {
            auto msg = std::make_shared<StringMsg>();
            msg->data = response;
            stream_tts_publisher_->publish(*msg);
        }
        if (response == "好的，我将会执行这个任务")
        {
            std::cout << "[DONE] 好的，我将会执行这个任务" << std::endl;
            auto msg = std::make_shared<StringMsg>();
            msg->data = "[DONE]";
            stream_tts_publisher_->publish(*msg);
        }
    }
    // 退出前设置 stop_，通知 executor_thread 退出
    stop_.store(true);
    RCLCPP_INFO(this->get_logger(), "Chat loop exited.");
    // 等待 executor 线程
    if (executor_thread_.joinable())
    {
        executor_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "ChatNode exited gracefully.");
}

// 大模型非流失处理
std::string ChatNode::processQuery(const std::string &query)
{
    // 这里可以集成本地模型或调用http api
    if (query.find("name") != std::string::npos)
    {
        return "My name is Robo Assistant.";
    }
    // httplib::Client client("https://perception-openai.openai.azure.com");//
    httplib::Client client("https://perception-openai-japan.openai.azure.com");
    // 设置请求头
    httplib::Headers headers = {
        {"api-key", "xxxxxx"} // 注意：不是 Bearer，而是 api-key
    };

    json body = {
        {"messages", {{{"role", "user"}, {"content", query}}}},
        {"model", "gpt-4o"} // 在 Azure 中，model 通常是 deployment name
    };

    // 发起 POST 请求
    // 注意：URL 包含 query 参数 api-version
    std::string target = "/openai/deployments/gpt-4o/chat/completions?api-version=2024-05-01-preview";
    auto start = std::chrono::high_resolution_clock::now();
    auto res = client.Post(target, headers, body.dump(), "application/json");
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "耗时: " << duration.count() << " ms" << std::endl;
    // 处理响应
    if (res && res->status == 200)
    {
        // std::cout << "Success:\n" << res->body << std::endl;
        json j = json::parse(res->body);

        std::string content = j["choices"][0]["message"]["content"];
        // std::cout << "Reply" << content << std::endl;
        std::cout << std::endl;
        return "回答: " + content;
    }
    else
    {
        if (res)
        {
            std::cout << "Error: " << res->status << "\n"
                      << res->body << std::endl;
        }
        else
        {
            std::cout << "Request failed, no response (check network or URL)" << std::endl;
        }
    }
    return "I understand your query: " + query;
}

// 模拟大模型流式处理
std::string ChatNode::processQueryStream(const std::string &query)
{
    // 检查是否包含 "任务"
    if (query.find("任务") != std::string::npos)
    {
        auto msg = std::make_shared<StringMsg>();
        msg->data = "[START]";
        stream_tts_publisher_->publish(*msg);

        return "好的，我将会执行这个任务";
    }
    if(query.find("结束外教口语") != std::string::npos)
    {
        auto msg = std::make_shared<StringMsg>();
        msg->data = "[START]";
        stream_tts_publisher_->publish(*msg);

        agent_type_ = agent_type::chat;
        return "外教口语任务已结束，再见";

    }
    // 1,初始化客户端
    httplib::Client client("https://dashscope.aliyuncs.com");
    client.enable_server_certificate_verification(false); // 禁用证书验证，仅用于测试
    client.set_connection_timeout(60);
    client.set_read_timeout(60);
    client.set_write_timeout(60);

    // 构造header和body
    httplib::Headers headers = {
        {"Content-Type", "application/json"},
        {"Authorization", "Bearer sk-xxxxxx"},
        {"Accept", "text/event-stream"},
        {"User-Agent", "OpenAI/Python 1.0"}};

    json body;
    body["model"] = "qwen-flash";//qwen3-max
    // body["model"] = "qwen3-235b-a22b";
    // body["model"] = "qwen3-max";
    body["stream"] = true;
    body["temperature"] = 0.7;
    body["enable_search"] = true;
    // body["search_options"]["enable_search_extension"] = true;
    // body["search_options"]["forced_search"] = true;

    // 构建工具列表提供给大模型使用
    json tools = json::array();
    for (const auto &tool : mcp_tools_)
    {
        json tool_json;
        tool_json["type"] = "function";
        tool_json["function"]["name"] = tool.name;
        tool_json["function"]["description"] = tool.description;
        tool_json["function"]["parameters"] = tool.inputSchema;
        tools.push_back(tool_json);
    }
    // ✅ 1. 构建 messages 数组
    body["messages"] = json::array();

    if (body["messages"].is_array() && body["messages"].empty())
    {
        body["messages"].push_back(json{{"role", "system"},
                                        {"content", "You are a helpful assistant."}});
    }

    // 追加所有历史消息（控制长度）
    // int start_idx = std::max(0, (int)conversation_history_.size() - MAX_HISTORY_TURNS);
    if(agent_type_ == agent_type::foreign_teach)
    {
        for(size_t i = 0; i < foreign_conversion_history_.size(); i++)
        {
            body["messages"].push_back(foreign_conversion_history_[i]);
        }
        std::cout << "agent_type::foreign_teach body = " << body.dump() << std::endl;
    }
    else
    {
        for (size_t i = 0; i < conversation_history_.size(); ++i)
        {
            body["messages"].push_back(conversation_history_[i]);
        }
    }

    // ✅ 2. 添加当前用户输入
    if(query.empty())
    {

    }
    else {
        body["messages"].push_back(json{{"role", "user"},
                                    {"content", query}});
    }
    

    // 添加工具信息
    body["tools"] = tools;
    body["tool_choice"] = "auto";
    body["top_p"] = 0.9;
    // 发送流式请求
    std::string target = "/compatible-mode/v1/chat/completions";

    std::cout << "开始流式处理..." << std::endl;
    // std::cout << "body = " << body.dump() << std::endl;
    // StreamHandler handler;
    auto self = std::shared_ptr<ChatNode>(this, [](ChatNode *) { /* no-op deleter */ });
    StreamHandler handler(self);

    // 记录请求开始时间
    auto start_time = std::chrono::high_resolution_clock::now();

    // 用于存储工具调用信息
    std::string tool_name;
    json tool_args;
    bool tool_call_detected = false;
    std::string full_response;

    // 用于累积工具调用信息
    std::vector<json> tool_calls_accumulated;

    // 标记请求是否被中断
    bool request_interrupted = false;

    auto res = client.Post(
        target,
        headers,
        body.dump(),
        "application/json",
        [&](const char *data, size_t len)
        {
            // 记录请求结束时间并计算耗时
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            RCLCPP_INFO(this->get_logger(), "Stream API request to %s took %lld ms", target.c_str(), duration.count());
            // 检查是否收到停止信号
            if (stop_tts_.load())
            {
                // 保存部分响应到成员变量，供后续保存到历史记录
                partial_response_ = handler.accumulated_response;
                request_interrupted = true;
                return false; // 停止流式处理
            }
            handler(data, len);
            // std::cout << "handler.accumulated_response = " << handler.accumulated_response << std::endl;
            // 累积完整响应
            full_response += std::string(data, len);

            // 解析流式响应以检测工具调用
            std::string chunk(data, len);
            size_t pos = 0;
            while ((pos = chunk.find("\n\n")) != std::string::npos)
            {
                std::string event = chunk.substr(0, pos);
                chunk.erase(0, pos + 2);

                if (event.find("data: ") == 0)
                {
                    try
                    {
                        std::cout << "Event: " << event << std::endl;
                        std::string json_str = event.substr(6);
                        if (json_str.empty() || json_str == "[DONE]")
                            continue;

                        json response = json::parse(json_str);
                        if (response.contains("choices") && !response["choices"].empty())
                        {
                            auto choice = response["choices"][0];
                            RCLCPP_DEBUG(this->get_logger(), "Processing choice data");
                            // 检查是否有工具调用
                            if (choice.contains("delta") && choice["delta"].contains("tool_calls"))
                            {
                                auto tool_calls = choice["delta"]["tool_calls"];
                                if (!tool_calls.is_null() && tool_calls.is_array())
                                {
                                    for (auto &tool_call : tool_calls)
                                    {
                                        if (tool_call.contains("index"))
                                        {
                                            int index = tool_call["index"];
                                            RCLCPP_DEBUG(this->get_logger(), "Processing tool call at index: %d", index);
                                            // 确保数组足够大
                                            while (tool_calls_accumulated.size() <= (size_t)index)
                                            {
                                                tool_calls_accumulated.push_back(json::object());
                                            }

                                            // 累积工具调用信息
                                            if (tool_call.contains("id"))
                                            {
                                                tool_calls_accumulated[index]["id"] = tool_call["id"];
                                                RCLCPP_DEBUG(this->get_logger(), "Tool call ID: %s", tool_call["id"].dump().c_str());
                                            }
                                            if (tool_call.contains("type"))
                                            {
                                                tool_calls_accumulated[index]["type"] = tool_call["type"];
                                                RCLCPP_DEBUG(this->get_logger(), "Tool call type: %s", tool_call["type"].dump().c_str());
                                            }
                                            if (tool_call.contains("function"))
                                            {
                                                auto function = tool_call["function"];
                                                RCLCPP_DEBUG(this->get_logger(), "Processing function data");
                                                if (!tool_calls_accumulated[index].contains("function"))
                                                {
                                                    tool_calls_accumulated[index]["function"] = json::object();
                                                }
                                                if (function.contains("name"))
                                                {
                                                    tool_calls_accumulated[index]["function"]["name"] = function["name"];
                                                    RCLCPP_DEBUG(this->get_logger(), "Function name: %s", function["name"].dump().c_str());
                                                }
                                                // if (function.contains("arguments"))
                                                // {
                                                //     if (!tool_calls_accumulated[index]["function"].contains("arguments"))
                                                //     {
                                                //         tool_calls_accumulated[index]["function"]["arguments"] = "";
                                                //     }
                                                //     tool_calls_accumulated[index]["function"]["arguments"] =
                                                //         tool_calls_accumulated[index]["function"]["arguments"].get<std::string>() +
                                                //         function["arguments"].get<std::string>();
                                                //     RCLCPP_DEBUG(this->get_logger(), "Function arguments accumulated");    
                                                // }
                                                if (function.contains("arguments") && !function["arguments"].is_null())
                                                {
                                                    if (!tool_calls_accumulated[index]["function"].contains("arguments"))
                                                    {
                                                        tool_calls_accumulated[index]["function"]["arguments"] = "";
                                                    }
                                                    if (function["arguments"].is_string())
                                                    {
                                                        tool_calls_accumulated[index]["function"]["arguments"] =
                                                            tool_calls_accumulated[index]["function"]["arguments"].get<std::string>() +
                                                            function["arguments"].get<std::string>();
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }

                            // 检查是否是工具调用结束
                            if (choice.contains("finish_reason") && choice["finish_reason"] == "tool_calls")
                            {
                                tool_call_detected = true;
                            }
                        }
                    }
                    catch (...)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Error parsing JSON response.");
                        // 解析错误，继续处理
                    }
                }
            }

            // 检查是否包含工具调用
            std::string accumulated = handler.accumulated_response;
            size_t tool_call_start = accumulated.find("TOOL_CALL");
            if (tool_call_start != std::string::npos)
            {
                size_t tool_call_end = accumulated.find("[TOOL_CALL_END]", tool_call_start);
                if (tool_call_end != std::string::npos)
                {
                    std::string tool_call_str = accumulated.substr(tool_call_start + 11, tool_call_end - tool_call_start - 11);
                    size_t separator_pos = tool_call_str.find("[|]");
                    if (separator_pos != std::string::npos)
                    {
                        tool_name = tool_call_str.substr(0, separator_pos);
                        std::string args_str = tool_call_str.substr(separator_pos + 3);
                        try
                        {
                            tool_args = json::parse(args_str);
                            tool_call_detected = true;
                        }
                        catch (...)
                        {
                            RCLCPP_WARN(this->get_logger(), "Failed to parse tool arguments: %s", args_str.c_str());
                        }
                    }
                }
            }
            return true;
        });

    if (!res)
    {
        // std::cerr << "请求失败: " << httplib::to_string(res.error()) << std::endl;
        // return "请求失败";
        std::string error_str = httplib::to_string(res.error());
        RCLCPP_ERROR(this->get_logger(), "请求失败: %s", error_str.c_str());
        
        // 如果是因为中断导致的连接取消，则不返回错误信息，而是正常处理
        if (request_interrupted && res.error() == httplib::Error::Canceled) {
            RCLCPP_INFO(this->get_logger(), "Request was intentionally canceled due to interruption");
            // 检查是否被中断
            if (stop_tts_.load())
            {
                // 保存部分响应到历史记录
                if (!partial_response_.empty() || !current_query_.empty())
                {
                    // 添加用户问题到历史
                    if (conversation_history_.empty())
                    {
                        conversation_history_.push_back({{"role", "system"},
                                                         {"content", "You are a helpful assistant."}});
                    }
                    if (agent_type_ == agent_type::foreign_teach)
                    {
                        foreign_conversion_history_.push_back({{"role", "user"},
                                                               {"content", current_query_}});

                        // 添加AI部分响应到历史
                        foreign_conversion_history_.push_back({{"role", "assistant"},
                                                               {"content", partial_response_}});
                    }
                    else
                    {
                        conversation_history_.push_back({{"role", "user"},
                                                         {"content", current_query_}});

                        // 添加AI部分响应到历史
                        conversation_history_.push_back({{"role", "assistant"},
                                                         {"content", partial_response_}});
                    }

                    std::cout << "已保存部分响应到历史对话" << std::endl;
                }

                // 重置标志和部分响应
                stop_tts_.store(false);
                partial_response_.clear();

                return {};//响应被中断，返回空，避免播放出来 响应被中断
            }
        }
        
        return "请求失败: " + error_str;
    }

    if (res->status != 200)
    {
        std::string error_details = "错误状态码: " + std::to_string(res->status);
        if (!res->body.empty())
        {
            error_details += "\n响应内容: " + res->body;
        }
        std::cerr << "\n"
                  << error_details << std::endl;
        return "请求失败: " + error_details;
    }

    // 检查是否被中断
    if (stop_tts_.load())
    {
        // 保存部分响应到历史记录
        if (!partial_response_.empty() || !current_query_.empty())
        {
            // 添加用户问题到历史
            if (conversation_history_.empty())
            {
                conversation_history_.push_back({{"role", "system"},
                                                 {"content", "You are a helpful assistant."}});
            }
            conversation_history_.push_back({{"role", "user"},
                                             {"content", current_query_}});

            // 添加AI部分响应到历史
            conversation_history_.push_back({{"role", "assistant"},
                                             {"content", partial_response_}});

            std::cout << "已保存部分响应到历史对话" << std::endl;
        }

        // 重置标志和部分响应
        stop_tts_.store(false);
        partial_response_.clear();

        return "响应被中断";
    }

    // 如果检测到工具调用(新的流式格式)
    if (tool_call_detected && !tool_calls_accumulated.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Detected tool call via streaming API");
        RCLCPP_INFO(this->get_logger(), "Tool calls accumulated: %d", tool_calls_accumulated.size());
        int number = 0;
        for(const auto &it : tool_calls_accumulated)
        {
            RCLCPP_INFO(this->get_logger(), "队列中第 %d 个tool call: %s", number++, it.dump().c_str());
            std::cout << std::endl;
        }
        // 处理累积的工具调用
        for (const auto &tool_call : tool_calls_accumulated)
        {
            if (tool_call.contains("function"))
            {
                auto function = tool_call["function"];
                if (function.contains("name") && function.contains("arguments"))
                {
                    std::string name = function["name"];
                    std::string args_str = function["arguments"];

                    RCLCPP_INFO(this->get_logger(), "Tool call detected: %s with args: %s", name.c_str(), args_str.c_str());

                    try
                    {
                        json args = json::parse(args_str);
                        json replace_msg;    
                        // 确保对话历史中包含初始系统消息和用户消息
                        if (conversation_history_.empty())
                        {
                            conversation_history_.push_back({{"role", "system"},
                                                             {"content", "You are a helpful assistant."}});
                        }

                        // 总是添加当前用户消息到对话历史（如果还没有）
                        bool user_message_exists = false;
                        size_t user_message_index = 0;
                        for (size_t i = 0; i < conversation_history_.size(); ++i)
                        {
                            const auto &msg = conversation_history_[i];
                            if (msg.value("role", "") == "user" && msg.contains("content") && msg["content"].is_string() && msg["content"] == query)
                            {
                                user_message_exists = true;
                                user_message_index = i;
                                break;
                            }
                        }

                        if (!user_message_exists)
                        {
                            if (agent_type_ == agent_type::foreign_teach)
                            {
                                foreign_conversion_history_.push_back({{"role", "user"},
                                                                       {"content", query}});
                            }
                            else
                            {
                                conversation_history_.push_back({{"role", "user"},
                                                                 {"content", query}});
                            }
                        }

                        // 生成工具调用ID
                        std::string tool_call_id = "call_" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                                                std::chrono::high_resolution_clock::now().time_since_epoch())
                                                                                .count());

                        // 添加工具调用记录到对话历史中
                        json tool_call_msg = {
                            {"role", "assistant"},
                            {"tool_calls", json::array({{{"id", tool_call_id}, // 简单的ID生成方式
                                                         {"type", "function"},
                                                         {"function", {{"name", name}, {"arguments", args_str}}}}})}};

                        conversation_history_.push_back(tool_call_msg);
                        // std::cout << "Added tool call to conversation history" << std::endl;
                        RCLCPP_DEBUG(this->get_logger(), "Added tool call to conversation history"); // 从INFO改为DEBUG
                        // 调用工具
                        auto future_result = callMCPTool(name, args);
                        MCPToolResult result = future_result.get();

                        
                        // 判断返回的内容是否包含图像数据（以data:image/开头表示是图像数据）
                        bool has_image_content = (!result.isError &&
                                                  result.content.find("data:image/") != std::string::npos);

                        if (!has_image_content)
                        {
                            std::string raw_tool_response;
                            if (!result.isError)
                            {
                                raw_tool_response = result.content;
                            }
                            else
                            {
                                raw_tool_response = "Error calling tool: " + result.content;
                            }

                            // 构建工具响应消息，包含tool_call_id
                            json tool_response_msg = {
                                {"role", "tool"},
                                {"tool_call_id", tool_call_id},
                                {"name", name},
                                {"content", raw_tool_response}};

                            // 将工具结果添加到对话历史中
                            // conversation_history_.push_back(tool_response_msg);
                            replace_msg = tool_response_msg;
                            std::cout << "Added tool response to conversation history" << std::endl;
                            std::cout << "Raw tool response: " << tool_response_msg.dump() << std::endl;
                            std::cout << "Replace message: " << replace_msg.dump() << std::endl;
                            conversation_history_.push_back(replace_msg);
                            if(agent_type_ == agent_type::foreign_teach)
                            {
                                RCLCPP_INFO(this->get_logger(), "切换英语口语模式成功");
                            }
                            else {
                                std::string polished_response = processQueryStream("");
                                std::cout << "Polished response: " << polished_response << std::endl;
                            }   
                            
                            // // 将润色后的回复添加到对话历史中
                            // if (!polished_response.empty())
                            // {
                            //     // conversation_history_.push_back({{"role", "assistant"},
                            //     //                                  {"content", polished_response}});
                            //     replace_msg["content"] = polished_response;
                            //     conversation_history_.push_back(replace_msg);
                            // }
                        }
                        else
                        {
                            std::cout << "tool返回的结果中有图片，特殊处理" << std::endl;
                            // 包含图像内容，特殊处理
                            // 添加提示信息
                            json prompt_msg = {
                                {"role", "tool"},
                                {"tool_call_id", tool_call_id},
                                {"name", name},
                                {"content", "Required images will be provided in next user input."}};
                            // replace_msg = prompt_msg;
                            conversation_history_.push_back(prompt_msg);

                            // 处理图像数据（支持单张和多张图片）
                            json image_contents = json::array();

                            try
                            {
                                // 尝试解析为JSON数组（多张图片情况）
                                json images_array = json::parse(result.content);
                                if (images_array.is_array())
                                {
                                    // 处理多张图片
                                    for (const auto &image_data : images_array)
                                    {
                                        if (image_data.is_string())
                                        {
                                            json image_content = {
                                                {"type", "image_url"},
                                                {"image_url", {{"url", image_data.get<std::string>()}}}};
                                            image_contents.push_back(image_content);
                                        }
                                    }
                                    // std::cout << "多张图片格式：" << image_contents << std::endl;
                                }
                                else
                                {
                                    // 不是数组，当作单张图片处理
                                    throw std::runtime_error("Not an array");
                                }
                            }
                            catch (...)
                            {
                                // 解析失败或不是数组，当作单张图片处理
                                json image_content = {
                                    {"type", "image_url"},
                                    {"image_url", {{"url", result.content}}}};
                                image_contents.push_back(image_content);
                            }
                            // std::string image_contents_str = conversation_history_.back().at("content").get<std::string>();
                            std::string image_contents_str = query;
                            std::cout << "image_contents_str: " << image_contents_str << std::endl;
                            json image_text = {
                                    {"type", "text"}, 
                                    {"text", query}
                                };
                            image_contents.push_back(image_text);
                            // 添加图像数据
                            json image_msg = {
                                {"role", "user"},
                                {"content", image_contents}};
                            // conversation_history_.push_back(image_msg);
                            open_ai_conversion_history_.clear();
                            open_ai_conversion_history_.push_back(image_msg);
                            std::cout << "Current conversation history size: " << conversation_history_.size() << std::endl;

                            // 使用主流程继续处理，将工具调用结果润色后返回
                            // 调用processQueryWithHistory方法基于更新后的对话历史生成润色后的回复
                            std::string polished_response = processQueryWithGpt(has_image_content);
                            // std::string polished_response = processQueryWithHistory(has_image_content);
                            std::cout << "Polished response: " << polished_response << std::endl;
                            // 将润色后的回复添加到对话历史中
                            if (!polished_response.empty())
                            {
                                conversation_history_.push_back({{"role", "assistant"},
                                                                 {"content", polished_response}});
                                // replace_msg["content"] = polished_response;
                                // conversation_history_.push_back(replace_msg);
                            }
                        }

                        // 返回特殊标记，表示TTS消息已发送，避免mainLoop中重复发送
                        return "[TTS_MESSAGE_SENT]";
                    }
                    catch (const std::exception &e)
                    {
                        std::string error_msg = "Exception while calling tool: " + std::string(e.what());
                        RCLCPP_ERROR(this->get_logger(), "%s", error_msg.c_str());
                        return error_msg;
                    }
                }
            }
        }
    }

    // // 发送剩余的TTS文本（如果有）
    // if (handler.node && !handler.tts_buffer.empty())
    // {
    //     auto msg = std::make_shared<StringMsg>();
    //     msg->data = handler.tts_buffer;
    //     handler.node->getStreamTTSPublisher()->publish(*msg);
    //     handler.tts_buffer.clear();
    // }

    // std::cout << "\n流式处理完成。" << std::endl;

    // ✅  同时把用户问题也加入历史（如果还没加）
    // 检查是否已经存在用户消息，避免重复添加
    bool user_message_exists = false;
    for (const auto &msg : conversation_history_)
    {
        if (msg.value("role", "") == "user" && msg.contains("content") && msg["content"].is_string() && msg["content"] == query)
        {
            user_message_exists = true;
            break;
        }
    }

    if (!user_message_exists)
    {
        if (conversation_history_.empty())
        {
            conversation_history_.push_back({{"role", "system"},
                                             {"content", "You are a helpful assistant."}});
        }
        if (agent_type_ == agent_type::foreign_teach)
        {
            foreign_conversion_history_.push_back({{"role", "assistant"},
                                                   {"content", query}});
        }
        else
        {
            conversation_history_.push_back({{"role", "user"},
                                             {"content", query}});
        }
    }

    // ✅  [重要] 在流结束后，将完整回复加入历史
    if (!handler.accumulated_response.empty())
    {
        if (agent_type_ == agent_type::foreign_teach)
        {
            foreign_conversion_history_.push_back({{"role", "assistant"},
                                                   {"content", handler.accumulated_response}});
        }
        else
        {
            conversation_history_.push_back({{"role", "assistant"},
                                             {"content", handler.accumulated_response}});
        }
    }

    std::cout << "当前对话轮数: " << conversation_history_.size() / 2 << std::endl;
    for (size_t i = 0; i < conversation_history_.size(); ++i)
    {
        // std::cout << "[" << i << "] " << conversation_history_[i] << std::endl;
        std::cout << "[" << i << "] ";
        const auto &msg = conversation_history_[i];
        if (msg.contains("content") && msg["content"].is_array())
        {
            // 特殊处理content为数组的情况（如图像消息）
            std::cout << "{";
            for (auto it = msg.begin(); it != msg.end(); ++it)
            {
                std::cout << "\"" << it.key() << "\": ";
                if (it.key() == "content" && it.value().is_array())
                {
                    std::cout << "[array with " << it.value().size() << " items]";
                }
                else
                {
                    std::cout << it.value();
                }
                if (std::next(it) != msg.end())
                {
                    std::cout << ", ";
                }
            }
            std::cout << "}" << std::endl;
        }
        else
        {
            // 普通情况直接输出
            std::cout << msg << std::endl;
        }
    }
    return "";
}

// 大模型输出结果流式响应处理器

void StreamHandler::operator()(const char *data, size_t len)
{
    std::string chunk(data, len);
    // std::cout << "Raw chunk received: " << chunk << std::endl;
    // 处理服务器发送的事件流数据
    size_t pos = 0;
    while ((pos = chunk.find("\n\n")) != std::string::npos)
    {
        std::string event = chunk.substr(0, pos);
        chunk.erase(0, pos + 2);

        if (event.empty() || event == "data: [DONE]")
        {
            // 🔔 关键：发布 [DONE] 到 ROS2 话题
            if (node && node->getStreamTTSPublisher())
            {
                // 发送剩余的TTS文本（如果有）
                if (!tts_buffer.empty())
                {
                    auto msg = std::make_shared<StringMsg>();
                    msg->data = tts_buffer;
                    node->getStreamTTSPublisher()->publish(*msg);
                    tts_buffer.clear();
                }
                auto done_msg = std::make_shared<StringMsg>();
                done_msg->data = "[DONE]";
                node->getStreamTTSPublisher()->publish(*done_msg);
            }
            continue;
        }

        if (event.find("data: ") == 0)
        {
            try
            {
                std::string json_str = event.substr(6);
                if (json_str.empty() || json_str == "[DONE]")
                    continue;

                json response = json::parse(json_str);
                if (response.contains("choices") && !response["choices"].empty())
                {
                    auto content = response["choices"][0]["delta"]["content"];
                    if (!content.is_null())
                    {
                        if (response_start)
                        {
                            if (node && node->getStreamTTSPublisher())
                            {
                                auto msg = std::make_shared<StringMsg>();
                                msg->data = "[START]";
                                node->getStreamTTSPublisher()->publish(*msg);
                            }
                            response_start = false;
                        }
                        // std::cout << content.get<std::string>() << std::flush;
                        std::string content_str = content.get<std::string>();
                        std::cout << content_str << std::flush;

                        // 累积响应内容
                        accumulated_response += content_str;

                        // 累积TTS文本内容
                        tts_buffer += content_str;

                        // 检查是否收到停止信号
                        if (node && node->stop_tts_.load())
                        {
                            // 发布已累积的内容然后退出
                            if (!tts_buffer.empty() && node->getStreamTTSPublisher())
                            {
                                auto msg = std::make_shared<StringMsg>();
                                msg->data = tts_buffer;
                                node->getStreamTTSPublisher()->publish(*msg);
                                tts_buffer.clear();
                            }
                            // 发布DONE信号
                            if (node->getStreamTTSPublisher())
                            {
                                auto done_msg = std::make_shared<StringMsg>();
                                done_msg->data = "[DONE]";
                                node->getStreamTTSPublisher()->publish(*done_msg);
                            }
                            return; // 退出处理函数
                        }

                        // 检查新增内容中是否包含句子结尾符号
                        bool sentence_found = false;
                        for (size_t i = last_check_pos; i < tts_buffer.length(); ++i)
                        {
                            char c = tts_buffer[i];
                            if (c == '.' || c == ',' || c == '，' || c == ';' || 
                                c == '；' || c == ':' || c == '：' || c == '。' || 
                                c == '?' || c == '？' || c == '!' || c == '！')
                            {
                                // 提取从开始到当前符号位置的子字符串（包含符号）
                                std::string sentence = tts_buffer.substr(0, i + 1);
                                // 发布完整句子到/interaction_tts_text话题
                                if (node && node->getStreamTTSPublisher() && !sentence.empty())
                                {
                                    auto msg = std::make_shared<StringMsg>();
                                    msg->data = sentence;
                                    node->getStreamTTSPublisher()->publish(*msg);
                                }
                                // 更新tts_buffer为剩余内容
                                tts_buffer = tts_buffer.substr(i + 1);
                                // 重置检查位置
                                last_check_pos = 0;
                                sentence_found = true;
                                // 重新开始循环，从头检查新内容
                                i = -1; // 循环结束后会++变0
                            }
                        }
                        
                        // 更新下次检查的起始位置
                        if (sentence_found) {
                            last_check_pos = 0;
                        } else {
                            last_check_pos = tts_buffer.length();
                        }
                        
                        // 检查是否需要发送累积的文本（超过2秒未发送或文本超过50字符）
                        auto now = std::chrono::steady_clock::now();
                        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_send_time).count();
                        if ((duration > 1.6 && !tts_buffer.empty()) || tts_buffer.length() > 20) {
                            if (node && node->getStreamTTSPublisher() && !tts_buffer.empty()) {
                                auto msg = std::make_shared<StringMsg>();
                                msg->data = tts_buffer;
                                node->getStreamTTSPublisher()->publish(*msg);
                                tts_buffer.clear();
                                last_check_pos = 0;
                                last_send_time = now;
                            }
                        }
                    }
                }
            }
            catch (const json::parse_error &e)
            {
                std::cerr << "\nJSON parse error: " << e.what() << std::endl;
                std::cerr << "Raw event: " << event << std::endl;
            }
        }

    }
    
}
// 初始化MCP工具
void ChatNode::initializeMCPTools()
{
    // 注册工具
    // 新增天气查询工具
    MCPTool get_weather_tool;
    get_weather_tool.name = "get_realtime_weather";
    get_weather_tool.description = "Get the realtime weather for a location.";
    get_weather_tool.inputSchema = {
        {"type", "object"},
        {"properties", {{"lng", {{"type", "number"}, {"description", "The longitude of the location to get the weather for"}}}, {"lat", {{"type", "number"}, {"description", "The latitude of the location to get the weather for"}}}}},
        {"required", {"lng", "lat"}}};
    get_weather_tool.callback = [this](const json &arguments)
    {
        return getWeather(arguments);
    };

    // 新增准备作业辅导工具
    MCPTool prepare_homework_tool;
    prepare_homework_tool.name = "prepare_for_homework_teaching";
    prepare_homework_tool.description = "Prepare for homework teaching task. This tool will adjust robot height and stretch out the arm to take photos of papers on the desk.";
    prepare_homework_tool.inputSchema = {
        {"type", "object"},
        {"properties", json::object()},
        {"required", json::array()}};
    prepare_homework_tool.callback = [this](const json &arguments)
    {
        return prepareForHomeworkTeaching(arguments);
    };

    // 新增执行作业辅导工具
    MCPTool perform_homework_tool;
    perform_homework_tool.name = "perform_homework_teaching";
    perform_homework_tool.description = "进行\"作业辅导\"任务。这个tool仅仅应当在需要获取桌面上试卷照片时调用；或者当用户需要你辅导作业时，直接调用这个tool获取作业的照片；其他情况不应当调用这个tool。\n对于获取的作业图片，图片可能旋转了某个角度，请从图片中识别出数学题目以及手写的答案（手写答案以'答：'开头）。首先输出识别到的题目，然后根据题目给出简洁明了的解题步骤和最终答案。接着，请检查提供的手写答案是否正确，并明确指出答案是正确还是错误，如果有误，请提供正确的答案。不要使用Markdown格式，也不要包含反引号，确保输出为纯中文可读格式。";
    perform_homework_tool.inputSchema = {
        {"type", "object"},
        {"properties", json::object()},
        {"required", json::array()}};
    perform_homework_tool.callback = [this](const json &arguments)
    {
        return performHomeworkTeaching(arguments);
    };

    // 新增结束作业辅导工具
    MCPTool end_homework_tool;
    end_homework_tool.name = "end_homework_teaching";
    end_homework_tool.description = "End the homework teaching task. This tool will draw the arms and lower the robot height.";
    end_homework_tool.inputSchema = {
        {"type", "object"},
        {"properties", json::object()},
        {"required", json::array()}};
    end_homework_tool.callback = [this](const json &arguments)
    {
        return endHomeworkTeaching(arguments);
    };

    // test_pic
    MCPTool test_pic;
    test_pic.name = "get_picture";
    test_pic.description = "解答图片中的数序题，并给出简短的解答过程。";
    test_pic.inputSchema = {
        {"type", "object"},
        {"properties", json::object()},
        {"required", json::array()}};
    test_pic.callback = [this](const json &arguments)
    {
        return getArmImage(arguments);
    };

    //新增观察工具
    MCPTool robot_watch_tool;
    robot_watch_tool.name = "robot_watch";
    robot_watch_tool.description = "This is a robot action tool which can serve as the eyes of the robot. If you need to see something or be asked about what you can see, invoke this tool directly. Take a photo using the front camera of the robot, to obtain the information of the surroundings. You can get images around a furniture using this action after you have moved near the furniture, and use these images for subsequent analysis. 你可以使用这个tool 得到附近环境或者某个家具附近的视觉信息，回答用户相关的问题。 需要注意的是如果想要使用这个工具得到某个家具附近的信息，你需要首先移动到那个家具附近。另外如果你被询问能看到什么时，应该直接调用这个工具。如果你看到了某个人类，请根据人物特征对人物进行一些夸奖，夸奖辞藻华丽丰富一些，对画面中人类的外貌、气质、穿搭、形象四个方面进行夸赞。必须使用比喻，排比，引用等修辞手法之一，进行夸赞。";
    robot_watch_tool.inputSchema = {
        {"type", "object"},
        {"properties", json::object()},
        {"required", json::array()}};
    robot_watch_tool.callback = [this](const json &arguments)
    {
        return robotWatch(arguments);
    };

    //新增观察工具
    MCPTool foreign_teach_tool;
    foreign_teach_tool.name = "foreign_teach";
    foreign_teach_tool.description = "进行“外教口语”任务。这个tool会持续执行和用户交流，最后退出“外教口语”任务。";
    foreign_teach_tool.inputSchema = {
        {"type", "object"},
        {"properties", json::object()},
        {"required", json::array()}};
    foreign_teach_tool.callback = [this](const json &arguments)
    {
        return foreginTeaching(arguments);
    };

    // 添加获取家具列表工具
    MCPTool get_furnitures_tool;
    get_furnitures_tool.name = "get_furnitures";
    get_furnitures_tool.description = "Return furniture list in home.";
    get_furnitures_tool.inputSchema = {
        {"type", "object"},
        {"properties", json::object()},
        {"required", json::array()}
    };
    get_furnitures_tool.callback = [this](const json &arguments) {
        return getFurnitures(arguments);
    };

    //移动到家具旁边
    MCPTool move_near_furniture_tool;
    move_near_furniture_tool.name = "move_near_furniture";
    move_near_furniture_tool.description = "The robot move near a piece of furniture. The argument furniture_name must be within this given furniture list: {furniture_list}. 如果你无法根据用户的输入选定该列表中一个合适的家具，不要调用这个tool。 Args: furniture_name: The name of the target furniture. 注意：移动到家具附近后，如果需要查看周围环境，必须调用 robot_watch 工具。";
    move_near_furniture_tool.inputSchema = {
        {"type", "object"},
        {"properties", {{"furniture_name", {{"type", "string"}}}}},
        {"required", {"furniture_name"}}};
    move_near_furniture_tool.callback = [this](const json &arguments)
    {
        return moveNearFurniture(arguments);
    };

    // 新增查询时间工具
    MCPTool get_current_time_tool;
    get_current_time_tool.name = "get_current_time";
    get_current_time_tool.description = "查询当前的日期和时间。当用户询问现在的时间、日期或需要时间信息时调用此工具。";
    get_current_time_tool.inputSchema = {
        {"type", "object"},
        {"properties", json::object()},
        {"required", json::array()}};
    get_current_time_tool.callback = [this](const json &arguments)
    {
        return getCurrentTime(arguments);
    };
    mcp_tools_.push_back(get_weather_tool);      // 添加天气查询工具
    mcp_tools_.push_back(prepare_homework_tool); // 添加准备作业辅导工具
    mcp_tools_.push_back(perform_homework_tool); // 添加执行作业辅导工具
    mcp_tools_.push_back(end_homework_tool);     // 添加结束作业辅导工具
    mcp_tools_.push_back(robot_watch_tool);      // 观察工具
    mcp_tools_.push_back(get_furnitures_tool);   // 添加获取家具列表工具
    // 测试
    // mcp_tools_.push_back(test_pic); // 测试
    mcp_tools_.push_back(move_near_furniture_tool);
    mcp_tools_.push_back(get_current_time_tool); //查询当前时间
    mcp_tools_.push_back(foreign_teach_tool);
    // 建立工具名称到索引的映射
    for (size_t i = 0; i < mcp_tools_.size(); ++i)
    {
        tool_name_to_index_[mcp_tools_[i].name] = i;
    }

    RCLCPP_INFO(this->get_logger(), "Initialized %zu MCP tools", mcp_tools_.size());
}

// 调用MCP工具
std::future<MCPToolResult> ChatNode::callMCPTool(const std::string &tool_name, const json &arguments)
{
    std::promise<MCPToolResult> promise;
    auto future = promise.get_future();

    // 查找工具
    auto it = tool_name_to_index_.find(tool_name);
    if (it == tool_name_to_index_.end())
    {
        MCPToolResult result;
        result.isError = true;
        result.content = "Tool not found: " + tool_name;
        promise.set_value(result);
        return future;
    }

    // 调用工具
    try
    {
        auto result = mcp_tools_[it->second].callback(arguments);
        promise.set_value(result.get());
    }
    catch (const std::exception &e)
    {
        MCPToolResult result;
        result.isError = true;
        result.content = "Error calling tool " + tool_name + ": " + e.what();
        promise.set_value(result);
    }

    return future;
}

// 获取可用工具列表
std::vector<MCPTool> ChatNode::getAvailableTools()
{
    return mcp_tools_;
}

// 具体的MCP工具实现

// 天气查询工具实现
std::future<MCPToolResult> ChatNode::getWeather(const json &arguments)
{
    std::promise<MCPToolResult> promise;

    try
    {
        // 获取经纬度参数
        double longitude = arguments.value("lng", 0.0);
        double latitude = arguments.value("lat", 0.0);

        // 创建HTTP客户端
        httplib::Client client("https://api.caiyunapp.com");
        client.enable_server_certificate_verification(false);
        client.set_connection_timeout(30);
        client.set_read_timeout(150);

        // 构造请求URL和参数
        std::string api_token = "AggoE3YT5cULLfcR";
        std::string path = "/v2.6/" + api_token + "/" + std::to_string(longitude) + "," + std::to_string(latitude) + "/realtime?lang=en_US";

        // 发送请求
        auto res = client.Get(path);

        if (!res)
        {
            MCPToolResult result;
            result.isError = true;
            result.content = "Failed to connect to weather service";
            promise.set_value(result);
            return promise.get_future();
        }

        if (res->status != 200)
        {
            MCPToolResult result;
            result.isError = true;
            result.content = "Weather service error: " + std::to_string(res->status);
            promise.set_value(result);
            return promise.get_future();
        }

        // 解析JSON响应
        json response = json::parse(res->body);
        json result_data = response["result"]["realtime"];

        // 提取关键天气信息并格式化为简洁的字符串
        double temperature = result_data["temperature"].get<double>();
        std::string skycon = result_data["skycon"].get<std::string>();
        double humidity = result_data["humidity"].get<double>() * 100;
        std::string skycon_desc;

        // 将天气代码转换为描述
        if (skycon == "CLEAR_DAY")
        {
            skycon_desc = "晴天";
        }
        else if (skycon == "PARTLY_CLOUDY_DAY" || skycon == "PARTLY_CLOUDY_NIGHT")
        {
            skycon_desc = "多云";
        }
        else if (skycon == "CLOUDY")
        {
            skycon_desc = "阴天";
        }
        else if (skycon == "RAIN")
        {
            skycon_desc = "下雨";
        }
        else if (skycon == "SNOW")
        {
            skycon_desc = "下雪";
        }
        else
        {
            skycon_desc = "部分多云";
        }

        // 构建简洁的天气信息
        std::string weather_info = "温度 " + std::to_string((int)temperature) + " 度，" +
                                   skycon_desc + "，湿度 " + std::to_string((int)humidity) + "%";

        MCPToolResult result;
        // std::cout<< "==================" << result_data.dump() << "==================" << std::endl;
        result.isError = false;
        result.content = weather_info;
        promise.set_value(result);
    }
    catch (const std::exception &e)
    {
        MCPToolResult result;
        result.isError = true;
        result.content = "Error getting weather: " + std::string(e.what());
        promise.set_value(result);
    }

    return promise.get_future();
}

// ... existing code ...
// 初始化对话历史
void ChatNode::initializeConversationHistory()
{
    // 打印当前工作目录
    // char current_path[FILENAME_MAX];
    // if (getcwd(current_path, sizeof(current_path)))
    // {
    //     RCLCPP_INFO(this->get_logger(), "Current working directory: %s", current_path);
    // }
    // else
    // {
    //     RCLCPP_WARN(this->get_logger(), "Failed to get current working directory");
    // }
    // 读取角色提示文件
    std::ifstream file("./src/chat_robot_cpp/prompt/role_prompt.txt");
    if (file.is_open())
    {
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string role_prompt = buffer.str();
        file.close();

        // 将角色提示添加到对话历史中
        conversation_history_.push_back({{"role", "system"},
                                         {"content", role_prompt}});
        RCLCPP_INFO(this->get_logger(), "Role prompt loaded successfully");
    }
    else
    {
        // 如果无法读取文件，使用默认提示
        conversation_history_.push_back({{"role", "system"},
                                         {"content", "You are a helpful assistant."}});
        RCLCPP_WARN(this->get_logger(), "Failed to load role prompt file, using default prompt");
    }
}

void ChatNode::initializeExternalRolePrompt()
{
    // 读取角色提示文件
    std::ifstream file("./src/chat_robot_cpp/prompt/foreign_teachers.txt");
    if (file.is_open())
    {
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string role_prompt = buffer.str();
        file.close();

        // 将角色提示添加到对话历史中
        foreign_conversion_history_.push_back({{"role", "system"},
                                         {"content", role_prompt}});
        RCLCPP_INFO(this->get_logger(), "Role prompt loaded successfully");
    }
    else
    {
        // 如果无法读取文件，使用默认提示
        foreign_conversion_history_.push_back({{"role", "system"},
                                         {"content", "You are a helpful assistant."}});
        RCLCPP_WARN(this->get_logger(), "Failed to load role prompt file, using default prompt");
    }
}

// 清除对话历史
void ChatNode::clearHistory()
{
    // 保留系统角色提示，清除其他对话历史
    if (!conversation_history_.empty())
    {
        auto system_prompt = conversation_history_[0];
        conversation_history_.clear();
        conversation_history_.push_back(system_prompt);
    }
}
//

// 润色工具响应
std::string ChatNode::polishToolResponse(const std::string &tool_name, const std::string &arguments, const std::string &raw_response)
{
    try
    {
        // 构造一个润色请求给大模型
        std::string polish_prompt = "Tool '" + tool_name + "' was called with arguments: " + arguments + "\n";
        polish_prompt += "The raw result is: " + raw_response + "\n";
        polish_prompt += "Please format this result in a user-friendly way. If the result is already well-formatted, just return it as is.";

        // 创建一个新的请求给大模型进行润色
        json polish_body;
        polish_body["model"] = "gpt-4o";
        polish_body["stream"] = false;
        polish_body["temperature"] = 0.7;

        // 构建消息历史
        json messages = json::array();
        messages.push_back(json{{"role", "system"}, {"content", "You are a helpful assistant that formats technical data in a user-friendly way. You make complex information easy to understand for users. If the information is already well-formatted, just return it as is."}});
        messages.push_back(json{{"role", "user"}, {"content", polish_prompt}});
        polish_body["messages"] = messages;

        // 发送润色请求
        httplib::Client polish_client("https://perception-openai-japan.openai.azure.com");
        polish_client.enable_server_certificate_verification(false);
        polish_client.set_connection_timeout(30);
        polish_client.set_read_timeout(150);

        httplib::Headers polish_headers = {
            {"Content-Type", "application/json"},
            {"api-key", "xxxxxx"}};

        std::string polish_target = "/openai/deployments/gpt-4o/chat/completions?api-version=2024-05-01-preview";
        auto polish_res = polish_client.Post(polish_target, polish_headers, polish_body.dump(), "application/json");

        if (polish_res && polish_res->status == 200)
        {
            json polish_response = json::parse(polish_res->body);
            std::cout << "1111111111111111111" << std::endl;
            return polish_response["choices"][0]["message"]["content"];
        }
        else
        {
            std::cout << "2222222222222222222" << std::endl;
            // 如果润色失败，使用原始数据
            return raw_response;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "Error polishing tool response: %s. Returning raw response.", e.what());
        // 如果润色过程中出现异常，使用原始数据
        return raw_response;
    }
}

// 使用当前对话历史处理查询
std::string ChatNode::processQueryWithHistory(bool has_image)
{
    // 1,初始化客户端
    httplib::Client client("https://dashscope.aliyuncs.com");
    client.enable_server_certificate_verification(false);
    client.set_connection_timeout(60);
    client.set_read_timeout(150);
    client.set_write_timeout(60);

    // 构造header和body
    httplib::Headers headers = {
        {"Content-Type", "application/json"},
        {"Authorization", "Bearer sk-xxxxxxxx"},
        {"Accept", "text/event-stream"},
        {"User-Agent", "OpenAI/Python 1.0"}};

    json body;
    body["model"] = "qwen-vl-max";
    body["stream"] = true;
    body["temperature"] = 0.7;

    // 构建工具列表提供给大模型使用
    json tools = json::array();
    for (const auto &tool : mcp_tools_)
    {
        json tool_json;
        tool_json["type"] = "function";
        tool_json["function"]["name"] = tool.name;
        tool_json["function"]["description"] = tool.description;
        tool_json["function"]["parameters"] = tool.inputSchema;
        tools.push_back(tool_json);
    }

    // 使用当前对话历史
    if(!has_image)
    {
         body["messages"] = conversation_history_;
    }
    else
    {
        // body["messages"] = open_ai_conversion_history_;
        // 先清空body["messages"]
        body["messages"] = json::array();

        // 追加所有conversation_history_消息
        for (size_t i = 0; i < conversation_history_.size(); ++i)
        {
            body["messages"].push_back(conversation_history_[i]);
        }

        // 再追加所有open_ai_conversion_history_消息
        for (size_t i = 0; i < open_ai_conversion_history_.size(); ++i)
        {
            body["messages"].push_back(open_ai_conversion_history_[i]);
        }
    }

    // 添加工具信息
    body["tools"] = tools;
    body["tool_choice"] = "auto";
    body["top_p"] = 0.9;
    std::cout << "body = " << body.dump() << std::endl;
    // 发送请求
    std::string target = "/compatible-mode/v1/chat/completions";

    // 记录请求开始时间
    auto start_time1 = std::chrono::high_resolution_clock::now();

    // 创建临时的流处理器用于处理润色结果
    auto self = std::shared_ptr<ChatNode>(this, [](ChatNode *) { /* no-op deleter */ });
    StreamHandler handler(self);
    std::string final_response;

     // 标记请求是否被中断
    bool request_interrupted = false;

    auto res = client.Post(
        target,
        headers,
        body.dump(),
        "application/json",
        [&](const char *data, size_t len)
        {
            auto end_time1 = std::chrono::high_resolution_clock::now();
            auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - start_time1);
            RCLCPP_INFO(this->get_logger(), "ProcessQueryWithHistory API request to %s took %lld ms", target.c_str(), duration1.count());
            // 检查是否收到停止信号
            if (stop_tts_.load())
            {
                request_interrupted = true;
                return false; // 停止流式处理
            }
            // 处理流式响应数据
            handler(data, len);
            return true;
        });

    // 检查是否被中断
    if (stop_tts_.load())
    {
        // 保存部分响应
        std::string partial_response = handler.accumulated_response;

        // 重置停止标志
        stop_tts_.store(false);

        // 返回已生成的部分内容
        return partial_response;
    }

    if (res && res->status == 200)
    {
        // 返回累积的响应内容
        return handler.accumulated_response;
    }
    else
    {
        if (res)
        {
            RCLCPP_ERROR(this->get_logger(), "Error calling LLM with history: %d", res->status);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to LLM service");
        }
        return "Sorry, I encountered an error processing your request.";
    }
    return "";
}


std::string ChatNode::processQueryWithGpt(bool has_image)
{
    // 1,初始化客户端
    httplib::Client client("https://perception-openai-japan.openai.azure.com");
    client.enable_server_certificate_verification(false);
    client.set_connection_timeout(60);
    client.set_read_timeout(150);
    client.set_write_timeout(60);

    // 构造header和body
    httplib::Headers headers = {
        {"Content-Type", "application/json"},
        {"api-key", "xxxxxxx"},
        {"Accept", "text/event-stream"}};

    json body;
    body["model"] = "gpt-4o";
    body["stream"] = true;

    // 构建工具列表提供给大模型使用
    json tools = json::array();
    for (const auto &tool : mcp_tools_)
    {
        json tool_json;
        tool_json["type"] = "function";
        tool_json["function"]["name"] = tool.name;
        tool_json["function"]["description"] = tool.description;
        tool_json["function"]["parameters"] = tool.inputSchema;
        tools.push_back(tool_json);
    }

    // 使用当前对话历史
    if(!has_image)
    {
         body["messages"] = conversation_history_;
    }
    else
    {
        // body["messages"] = open_ai_conversion_history_;
        // 先清空body["messages"]
        body["messages"] = json::array();

        // 追加所有conversation_history_消息
        for (size_t i = 0; i < conversation_history_.size(); ++i)
        {
            body["messages"].push_back(conversation_history_[i]);
        }

        // 再追加所有open_ai_conversion_history_消息
        for (size_t i = 0; i < open_ai_conversion_history_.size(); ++i)
        {
            body["messages"].push_back(open_ai_conversion_history_[i]);
        }
    }

    // 添加工具信息
    body["tools"] = tools;
    body["tool_choice"] = "auto";
    // body["top_p"] = 0.9;
    std::cout << "body = " << body.dump() << std::endl;
    // 发送请求
    std::string target = "/openai/deployments/gpt-4o/chat/completions?api-version=2024-05-01-preview";

    // 记录请求开始时间
    auto start_time1 = std::chrono::high_resolution_clock::now();

    // 创建临时的流处理器用于处理润色结果
    auto self = std::shared_ptr<ChatNode>(this, [](ChatNode *) { /* no-op deleter */ });
    StreamHandler handler(self);
    std::string final_response;

     // 标记请求是否被中断
    bool request_interrupted = false;

    auto res = client.Post(
        target,
        headers,
        body.dump(),
        "application/json",
        [&](const char *data, size_t len)
        {
            auto end_time1 = std::chrono::high_resolution_clock::now();
            auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - start_time1);
            RCLCPP_INFO(this->get_logger(), "processQueryWithGpt API request to %s took %lld ms", target.c_str(), duration1.count());
            // 检查是否收到停止信号
            if (stop_tts_.load())
            {
                request_interrupted = true;
                return false; // 停止流式处理
            }
            // 处理流式响应数据
            handler(data, len);
            return true;
        });

    // 检查是否被中断
    if (stop_tts_.load())
    {
        // 保存部分响应
        std::string partial_response = handler.accumulated_response;

        // 重置停止标志
        stop_tts_.store(false);

        // 返回已生成的部分内容
        return partial_response;
    }

    if (res && res->status == 200)
    {
        // 返回累积的响应内容
        return handler.accumulated_response;
    }
    else
    {
        if (res)
        {
            RCLCPP_ERROR(this->get_logger(), "Error calling LLM with history: %d", res->status);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to LLM service");
        }
        return "Sorry, I encountered an error processing your request.";
    }
    return "";
}
// 发送升降控制请求
std::future<MCPToolResult> ChatNode::sendLiftControllerRequest(int32_t height, int32_t velocity)
{
    std::cout << "===========sendLiftControllerRequest===========" << std::endl;
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    // 检查服务客户端是否可用
    if (!lift_controller_client_ || !lift_controller_client_->service_is_ready())
    {
        MCPToolResult result;
        result.isError = true;
        result.content = "Lift controller service is not available";
        promise->set_value(result);
        return future;
    }

    // 创建请求
    auto request = std::make_shared<LiftController::Request>();
    request->lift_pose = height;
    request->lift_vel = velocity;

    RCLCPP_INFO(this->get_logger(), "Sending lift controller request: height=%d, velocity=%d", height, velocity);

    // 发送请求
    std::function<void(rclcpp::Client<LiftController>::SharedFuture)> callback =
        [promise, request](rclcpp::Client<LiftController>::SharedFuture future)
    {
        std::cout << "std::function<void(rclcpp::Client<LiftController>::SharedFuture)> callback" << std::endl;
        MCPToolResult result;
        try
        {
            auto response = future.get();
            if (response->result)
            {
                result.isError = false;
                result.content = "Robot height adjusted successfully to " + std::to_string(request->lift_pose) + "mm";
                std::cout << "result.content: " << result.content << std::endl;
            }
            else
            {
                result.isError = true;
                result.content = "Failed to adjust robot height";
                std::cout << "result.content: " << result.content << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            result.isError = true;
            result.content = "Exception while adjusting robot height: " + std::string(e.what());
        }
        promise->set_value(result);
    };

    lift_controller_client_->async_send_request(request, callback);

    return future;
}

// 调整机器人高度的普通函数实现
std::future<MCPToolResult> ChatNode::adjustRobotHeight(const json &arguments)
{
    std::cout << "======================= adjustRobotHeight ============================" << std::endl;
    std::promise<MCPToolResult> promise;

    try
    {
        // 获取高度参数
        int32_t height = arguments.value("height", -1);

        // 验证参数 (假设有效高度范围是0-500mm)
        if (height < 0 || height > 500)
        {
            MCPToolResult result;
            result.isError = true;
            result.content = "Invalid height. Must be between 0 and 500 mm.";
            promise.set_value(result);
            return promise.get_future();
        }

        // 发送升降控制请求 (使用默认速度40)
        return sendLiftControllerRequest(height, 40);
    }
    catch (const std::exception &e)
    {
        MCPToolResult result;
        result.isError = true;
        result.content = "Error adjusting robot height: " + std::string(e.what());
        promise.set_value(result);
        return promise.get_future();
    }
}

// 机械臂伸缩控制函数实现
std::future<MCPToolResult> ChatNode::sendArmTask(int32_t task_id)
{
    std::cout << "sendArmTask" << std::endl;
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    // 检查action客户端是否可用
    if (!arm_task_client_ || !arm_task_client_->action_server_is_ready())
    {
        MCPToolResult result;
        result.isError = true;
        result.content = "Arm task action server is not available";
        promise->set_value(result);
        return future;
    }

    // 创建目标
    auto goal = ArmTask::Goal();
    goal.task_id = task_id;

    RCLCPP_INFO(this->get_logger(), "Sending arm task goal: task_id=%d", task_id);

    // 发送目标
    auto send_goal_options = rclcpp_action::Client<ArmTask>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [promise](rclcpp_action::ClientGoalHandle<ArmTask>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            MCPToolResult result;
            result.isError = true;
            result.content = "Arm task goal was rejected by server";
            promise->set_value(result);
        }
    };

    send_goal_options.result_callback =
        [promise, task_id](const rclcpp_action::ClientGoalHandle<ArmTask>::WrappedResult &result)
    {
        MCPToolResult tool_result;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            tool_result.isError = false;
            tool_result.content = "Arm task " + std::to_string(task_id) + " completed successfully";
            break;
        case rclcpp_action::ResultCode::ABORTED:
            tool_result.isError = true;
            tool_result.content = "Arm task " + std::to_string(task_id) + " was aborted";
            break;
        case rclcpp_action::ResultCode::CANCELED:
            tool_result.isError = true;
            tool_result.content = "Arm task " + std::to_string(task_id) + " was canceled";
            break;
        default:
            tool_result.isError = true;
            tool_result.content = "Unknown result code for arm task " + std::to_string(task_id);
            break;
        }
        promise->set_value(tool_result);
    };

    arm_task_client_->async_send_goal(goal, send_goal_options);

    return future;
}

// 准备作业辅导功能 - 注册为MCP工具
std::future<MCPToolResult> ChatNode::prepareForHomeworkTeaching(const json & /*arguments*/)
{
    std::cout << "========== prepareForHomeworkTeaching ===========" << std::endl;
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    // 创建一个线程来执行作业辅导准备任务，确保不阻塞主线程
    std::thread([this, promise]()
                {
        try {
            std::cout << "========== 创建一个线程来执行作业辅导准备任务，确保不阻塞主线程 ===========" << std::endl;
            // 1. 调整机器人高度到320mm（与Python代码保持一致）
            json height_args;
            height_args["height"] = 320;
            auto height_future = adjustRobotHeight(height_args);
            auto height_result = height_future.get();
            
            if (height_result.isError) {
                MCPToolResult result;
                result.isError = true;
                result.content = "Failed to adjust robot height: " + height_result.content;
                promise->set_value(result);
                return;
            }
            
            // 2. 伸出机械臂到44号位置（与Python代码保持一致）
            std::cout << "Extending arm to 44..." << std::endl;
            auto arm_future = sendArmTask(44);
            auto arm_result = arm_future.get();
            
            if (arm_result.isError) {
                MCPToolResult result;
                result.isError = true;
                result.content = "Failed to stretch out arms: " + arm_result.content;
                promise->set_value(result);
                return;
            }
            
            // 成功完成所有操作
            MCPToolResult result;
            result.isError = false;
            result.content = "Robot is ready for homework teaching. Height adjusted and arms stretched out.";
            promise->set_value(result);
        }
        catch (const std::exception &e) {
            MCPToolResult result;
            result.isError = true;
            result.content = "Error preparing for homework teaching: " + std::string(e.what());
            promise->set_value(result);
        } })
        .detach();

    return future;
}

// 结束作业辅导功能 - 注册为MCP工具
std::future<MCPToolResult> ChatNode::endHomeworkTeaching(const json & /*arguments*/)
{
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    // 创建一个线程来执行结束作业辅导任务，确保不阻塞主线程
    std::thread([this, promise]()
                {
        try {
            // 1. 机械臂收回（task_id=45，与Python代码保持一致）
            auto arm_future = sendArmTask(45);
            auto arm_result = arm_future.get();
            
            if (arm_result.isError) {
                MCPToolResult result;
                result.isError = true;
                result.content = "Failed to draw arms: " + arm_result.content;
                promise->set_value(result);
                return;
            }
            
            // 2. 降低机身高度到0mm（与Python代码保持一致）
            json height_args;
            height_args["height"] = 0;
            auto height_future = adjustRobotHeight(height_args);
            auto height_result = height_future.get();
            
            if (height_result.isError) {
                MCPToolResult result;
                result.isError = true;
                result.content = "Failed to lower robot height: " + height_result.content;
                promise->set_value(result);
                return;
            }
            
            // 成功完成所有操作
            MCPToolResult result;
            result.isError = false;
            result.content = "Robot has finished homework teaching. Arms drawn and height lowered.";
            promise->set_value(result);
        }
        catch (const std::exception &e) {
            MCPToolResult result;
            result.isError = true;
            result.content = "Error ending homework teaching: " + std::string(e.what());
            promise->set_value(result);
        } })
        .detach();

    return future;
}

// Base64编码函数
std::string ChatNode::base64_encode(const std::vector<uint8_t> &data)
{
    static const char *chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    // 预分配内存以提高性能
    std::string result;
    result.reserve(((data.size() / 3) + 1) * 4);

    int val = 0, valb = -6;

    for (uint8_t c : data)
    {
        val = (val << 8) + c;
        valb += 8;
        while (valb >= 0)
        {
            result.push_back(chars[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }

    if (valb > -6)
    {
        result.push_back(chars[((val << 8) >> (valb + 8)) & 0x3F]);
    }

    while (result.size() % 4)
    {
        result.push_back('=');
    }

    return result;
}

// 保存图像到文件
void ChatNode::saveImageToFile(const std::vector<uint8_t> &data, const std::string &filename)
{
    std::ofstream file(filename, std::ios::binary);
    if (file.is_open())
    {
        file.write(reinterpret_cast<const char *>(data.data()), data.size());
        file.close();
        RCLCPP_INFO(this->get_logger(), "Image saved to %s", filename.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to save image to %s", filename.c_str());
    }
}

// 保存图像到文件（重载版本，带格式参数）
void ChatNode::saveImageToFile(const std::vector<uint8_t> &data, const std::string &filename, const std::string &format)
{
    if (data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Received empty image data");
        return;
    }

    try {
        if (format == "bgr") {
            // 处理BGR格式的图像数据
            // 尝试推断图像尺寸
            int total_pixels = data.size() / 3; // BGR每个像素3个字节
            
            // 尝试几种常见的图像比例来推断尺寸
            // 常见比例: 4:3, 16:9, 3:2, 1:1
            std::vector<std::pair<int, int>> ratios = {{4, 3}, {16, 9}, {3, 2}, {1, 1}};
            
            int width = 0, height = 0;
            bool found = false;
            
            for (const auto& ratio : ratios) {
                int w_ratio = ratio.first;
                int h_ratio = ratio.second;
                
                // 根据比例计算可能的尺寸
                double area = static_cast<double>(total_pixels);
                double ratio_area = static_cast<double>(w_ratio * h_ratio);
                double scale = std::sqrt(area / ratio_area);
                
                int w = static_cast<int>(std::round(scale * w_ratio));
                int h = static_cast<int>(std::round(scale * h_ratio));
                
                // 检查是否匹配
                if (w * h * 3 == static_cast<int>(data.size())) {
                    width = w;
                    height = h;
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                RCLCPP_WARN(this->get_logger(), "Could not determine image dimensions. Data size: %zu bytes", data.size());
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Decoded BGR image: %dx%d", width, height);
            
            // 保存为PPM格式（P6表示二进制RGB格式）
            std::ofstream file(filename, std::ios::binary);
            if (file.is_open()) {
                // 写入PPM头部
                file << "P6\n" << width << " " << height << "\n255\n";
                
                // 写入像素数据（需要将BGR转换为RGB）
                for (size_t i = 0; i < data.size(); i += 3) {
                    // BGR -> RGB
                    file.put(data[i + 2]); // R
                    file.put(data[i + 1]); // G
                    file.put(data[i + 0]); // B
                }
                
                file.close();
                RCLCPP_INFO(this->get_logger(), "BGR image converted and saved to: %s (PPM format)", filename.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save converted image to: %s", filename.c_str());
            }
        } else {
            // 处理其他格式（如jpeg, png等）
            std::ofstream file(filename, std::ios::binary);
            if (file.is_open()) {
                file.write(reinterpret_cast<const char*>(data.data()), data.size());
                file.close();
                RCLCPP_INFO(this->get_logger(), "Image saved to: %s (format: %s)", 
                           filename.c_str(), format.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save image to: %s", filename.c_str());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception occurred while saving image: %s", e.what());
    }
}
// 获取机械臂摄像头图像
std::future<MCPToolResult> ChatNode::getArmImage(const json & /*arguments*/)
{
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    // 创建一个线程来获取图像，确保不阻塞主线程
    std::thread([this, promise]()
                {
        try {
            auto start_time_1 = std::chrono::high_resolution_clock::now();
            std::vector<uint8_t> image_data;
            CompressedImage::SharedPtr latest_image; // 将latest_image变量移到此处，使其在整个lambda函数作用域内可见
            
            std::cout << "从ROS主题订阅获取图像..." << std::endl;
            // 本地文件不存在，尝试从ROS主题订阅获取图像
            RCLCPP_INFO(this->get_logger(), "Trying to subscribe to image topic");
            
            // 创建图像订阅者 (使用CompressedImage消息)
            std::mutex image_mutex;
            std::condition_variable image_cv;
            bool image_received = false;
            rclcpp::Subscription<CompressedImage>::SharedPtr image_sub;
            
            // 订阅图像主题
            image_sub = this->create_subscription<CompressedImage>(
                "/camera_dcl_left/custom_cam_color_test",
                rclcpp::QoS(1).best_effort(),
                [&latest_image, &image_mutex, &image_cv, &image_received](const CompressedImage::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(image_mutex);
                    latest_image = msg;
                    image_received = true;
                    image_cv.notify_one();
                }
            );
            
            // 检查订阅是否创建成功
            if (!image_sub) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create image subscription");
                MCPToolResult result;
                result.isError = true;
                result.content = "Failed to create image subscription";
                promise->set_value(result);
                return;
            }
            
            // 等待图像接收
            std::unique_lock<std::mutex> lock(image_mutex);
            if (!image_cv.wait_for(lock, std::chrono::seconds(30), [&image_received]() { return image_received; })) {
                // 超时未收到图像
                image_sub.reset(); // 重置订阅者
                MCPToolResult result;
                result.isError = true;
                result.content = "Timeout waiting for arm camera image";
                promise->set_value(result);
                return;
            }
            
            // 成功接收到图像
            image_sub.reset(); // 重置订阅者
            // 检查图像数据
            if (!latest_image) {
                RCLCPP_ERROR(this->get_logger(), "Received null image message");
                MCPToolResult result;
                result.isError = true;
                result.content = "Received null image message";
                promise->set_value(result);
                return;
            }
            
            // 获取图像数据
            if (latest_image->data.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Received empty image data");
                MCPToolResult result;
                result.isError = true;
                result.content = "Received empty image data";
                promise->set_value(result);
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Received compressed image data, size: %zu bytes, format: %s", 
                       latest_image->data.size(), latest_image->format.c_str());
            
            // 保存图像到本地文件
            // 生成文件名
            auto now = std::chrono::system_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
                now.time_since_epoch()).count();
            
            // 保存为WebP格式
            std::string filename = "arm_camera_image_" + std::to_string(timestamp) + ".webp";
            
            // 处理BGR图像数据并转换为WebP
            if (latest_image->format == "bgr") {
                RCLCPP_INFO(this->get_logger(), "Processing BGR image data and converting to WebP");
                
                // 推断BGR图像尺寸
                int total_pixels = latest_image->data.size() / 3;
                std::vector<std::pair<int, int>> ratios = {{4, 3}, {16, 9}, {3, 2}, {1, 1}};
                int width = 0, height = 0;
                bool found = false;
                
                for (const auto& ratio : ratios) {
                    int w_ratio = ratio.first;
                    int h_ratio = ratio.second;
                    
                    double area = static_cast<double>(total_pixels);
                    double ratio_area = static_cast<double>(w_ratio * h_ratio);
                    double scale = std::sqrt(area / ratio_area);
                    
                    int w = static_cast<int>(std::round(scale * w_ratio));
                    int h = static_cast<int>(std::round(scale * h_ratio));
                    
                    if (w * h * 3 == static_cast<int>(latest_image->data.size())) {
                        width = w;
                        height = h;
                        found = true;
                        break;
                    }
                }
                
                if (!found) {
                    RCLCPP_WARN(this->get_logger(), "Could not determine image dimensions. Data size: %zu bytes", latest_image->data.size());
                    MCPToolResult result;
                    result.isError = true;
                    result.content = "Could not determine image dimensions";
                    promise->set_value(result);
                    return;
                }
                
                RCLCPP_INFO(this->get_logger(), "Decoded BGR image: %dx%d", width, height);
                
                // 使用libwebp将BGR数据编码为WebP
                uint8_t* webp_data = nullptr;
                size_t webp_size = 0;
                
                // 设置WebP编码参数
                WebPConfig config;
                if (!WebPConfigPreset(&config, WEBP_PRESET_PHOTO, 75.0f)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to configure WebP preset");
                    MCPToolResult result;
                    result.isError = true;
                    result.content = "Failed to configure WebP preset";
                    promise->set_value(result);
                    return;
                }
                
                // 配置更多参数
                config.method = 6;
                config.quality = 75.0f;
                config.target_size = 0;
                config.target_PSNR = 0;
                config.segments = 4;
                config.sns_strength = 50;
                config.filter_strength = 60;
                config.filter_sharpness = 0;
                config.filter_type = 1;
                config.autofilter = 1;
                config.alpha_compression = 1;
                config.alpha_filtering = 1;
                config.alpha_quality = 100;
                config.pass = 1;
                config.show_compressed = 0;
                config.preprocessing = 4;
                config.partitions = 3;
                config.partition_limit = 0;
                config.emulate_jpeg_size = 0;
                config.thread_level = 1;
                config.low_memory = 0;
                config.near_lossless = 100;
                config.exact = 0;
                config.use_delta_palette = 0;
                config.use_sharp_yuv = 1;
                
                // 检查配置是否有效
                if (!WebPValidateConfig(&config)) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid WebP configuration");
                    MCPToolResult result;
                    result.isError = true;
                    result.content = "Invalid WebP configuration";
                    promise->set_value(result);
                    return;
                }
                
                // 创建WebP图片对象
                WebPPicture pic;
                if (!WebPPictureInit(&pic)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to initialize WebP picture");
                    MCPToolResult result;
                    result.isError = true;
                    result.content = "Failed to initialize WebP picture";
                    promise->set_value(result);
                    return;
                }
                
                pic.width = width;
                pic.height = height;
                pic.use_argb = 1;
                
                // 将BGR数据导入到WebP图片对象
                if (!WebPPictureImportBGR(&pic, latest_image->data.data(), width * 3)) {
                    WebPPictureFree(&pic);
                    RCLCPP_ERROR(this->get_logger(), "Failed to import BGR data to WebP picture");
                    MCPToolResult result;
                    result.isError = true;
                    result.content = "Failed to import BGR data to WebP picture";
                    promise->set_value(result);
                    return;
                }
                
                // 分配内存用于存储编码后的WebP数据
                WebPMemoryWriter writer;
                WebPMemoryWriterInit(&writer);
                pic.writer = WebPMemoryWrite;
                pic.custom_ptr = &writer;
                
                // 执行编码
                int ok = WebPEncode(&config, &pic);
                if (!ok) {
                    WebPMemoryWriterClear(&writer);
                    WebPPictureFree(&pic);
                    RCLCPP_ERROR(this->get_logger(), "Failed to encode WebP image");
                    MCPToolResult result;
                    result.isError = true;
                    result.content = "Failed to encode WebP image";
                    promise->set_value(result);
                    return;
                }
                
                // 获取编码后的WebP数据
                webp_size = writer.size;
                webp_data = writer.mem;
                
                // 保存WebP图像到文件
                std::ofstream file(filename, std::ios::binary);
                if (file.is_open()) {
                    file.write(reinterpret_cast<const char*>(webp_data), webp_size);
                    file.close();
                    RCLCPP_INFO(this->get_logger(), "WebP image saved to: %s, size: %zu bytes", filename.c_str(), webp_size);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to save WebP image to: %s", filename.c_str());
                }
                
                // 将WebP数据复制到image_data向量中
                image_data.assign(webp_data, webp_data + webp_size);
                
                // 清理资源
                WebPMemoryWriterClear(&writer);
                WebPPictureFree(&pic);
            } else {
                // 对于非BGR格式，直接使用压缩图像数据并转换为WebP
                image_data = latest_image->data;
                
                // 保存原始图像到文件
                saveImageToFile(image_data, filename);
            }
            
            // 将图像数据编码为base64
            std::string base64_data = base64_encode(image_data);
            // 记录结束时间并计算耗时
            auto end_time_1 = std::chrono::high_resolution_clock::now();
            auto duration_1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_1 - start_time_1);

            // 构造完整的data URL (使用WebP格式)
            std::string mime_type = "image/webp";
            std::string data_url = "data:" + mime_type + ";base64," + base64_data;
            
            // 输出耗时信息
            RCLCPP_INFO(this->get_logger(), "Image processing completed. Total time: %ld ms, Final image size: %zu bytes", 
                       duration_1.count(), image_data.size());
            MCPToolResult result;
            result.isError = false;
            result.content = data_url;
            promise->set_value(result);
        }
        catch (const std::exception &e) {
            MCPToolResult result;
            result.isError = true;
            result.content = "Error getting arm image: " + std::string(e.what());
            promise->set_value(result);
        } })
        .detach();

    return future;
}
// 执行作业辅导功能 - 注册为MCP工具
std::future<MCPToolResult> ChatNode::performHomeworkTeaching(const json & /*arguments*/)
{
    std::cout << "执行作业辅导功能..." << std::endl;
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    // 创建一个线程来执行作业辅导任务，确保不阻塞主线程
    std::thread([this, promise]()
                {
        try {
            // 直接获取机械臂摄像头图像（与Python代码保持一致）
            auto image_future = getArmImage(json::object());
            auto image_result = image_future.get();
            
            if (image_result.isError) {
                MCPToolResult result;
                result.isError = true;
                result.content = "Failed to get arm image: " + image_result.content;
                promise->set_value(result);
                return;
            }
            
            // 成功完成操作，返回图像数据的base64编码
            MCPToolResult result;
            result.isError = false;
            result.content = image_result.content;  // 这里返回图像的base64编码
            promise->set_value(result);
        }
        catch (const std::exception &e) {
            MCPToolResult result;
            result.isError = true;
            result.content = "Error performing homework teaching: " + std::string(e.what());
            promise->set_value(result);
        } })
        .detach();

    return future;
}

// ... existing code ...
// 清除对话历史中的旧图像数据以提高性能
void ChatNode::cleanupOldImageMessages()
{
    // 只保留最近的2轮对话，减少图像数据传输
    if (conversation_history_.size() > 4)
    {
        // 保留系统消息和最近的几条消息
        std::vector<json> cleaned_history;

        // 始终保留系统消息（第一条）
        if (!conversation_history_.empty())
        {
            cleaned_history.push_back(conversation_history_[0]);
        }

        // 只保留最近的几条消息
        int start_idx = std::max(1, (int)conversation_history_.size() - 4);
        for (int i = start_idx; i < (int)conversation_history_.size(); ++i)
        {
            const auto &msg = conversation_history_[i];
            // 如果是图像消息，简化处理
            if (msg.contains("content") && msg["content"].is_array())
            {
                bool is_image_message = false;
                for (const auto &content_item : msg["content"])
                {
                    if (content_item.contains("type") && content_item["type"] == "image_url")
                    {
                        is_image_message = true;
                        break;
                    }
                }

                if (is_image_message)
                {
                    // 保留完整的图像消息，不进行清理
                    // 因为可能在后续请求中还需要使用这些图像
                    cleaned_history.push_back(msg);
                }
                else
                {
                    cleaned_history.push_back(msg);
                }
            }
            else
            {
                cleaned_history.push_back(msg);
            }
        }

        conversation_history_ = std::move(cleaned_history);
    }
}

// 实现 robotWatch 方法
std::future<MCPToolResult> ChatNode::robotWatch(const json & /*arguments*/)
{
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    // 创建一个线程来获取图像，确保不阻塞主线程
    std::thread([this, promise]()
                {
        try {
            // 定义云台角度列表，与Python版本保持一致
            std::vector<std::pair<float, float>> angle_list = {
                {-50.0f, 0.0f}, 
                {0.0f, 0.0f}, 
                {0.0f, -30.0f}, 
                {50.0f, 0.0f}
            };
            
            // 存储图像数据的数组
            std::vector<std::string> images;
            
            // 创建云台控制发布者
            auto gimble_publisher = this->create_publisher<serial_interfaces::msg::GimbleControl>("/gimble_control", 10);
            
            // 创建图像订阅者
            custom_image_msg::msg::Image4m::SharedPtr latest_image;
            std::mutex image_mutex;
            std::condition_variable image_cv;
            bool image_received = false;
            rclcpp::Subscription<custom_image_msg::msg::Image4m>::SharedPtr image_sub;
            
            auto start_time = std::chrono::high_resolution_clock::now(); // 记录开始时间
            // 订阅图像主题
            image_sub = this->create_subscription<custom_image_msg::msg::Image4m>(
                "/camera_dcw2/custom_cam_color",
                rclcpp::QoS(1).best_effort(),
                [&latest_image, &image_mutex, &image_cv, &image_received](const custom_image_msg::msg::Image4m::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(image_mutex);
                    latest_image = msg;
                    image_received = true;
                    image_cv.notify_one();
                });

            // 检查订阅是否创建成功
            if (!image_sub) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create front camera image subscription");
                MCPToolResult result;
                result.isError = true;
                result.content = "Failed to create front camera image subscription";
                promise->set_value(result);
                return;
            }
            
            // 遍历角度列表，控制云台并获取图像
            int image_index = 0;
            for (const auto& angle : angle_list) {
                float hori = angle.first;
                float vert = angle.second;
                
                // 控制云台到指定角度
                sendGimbleControl(hori, vert);
                
                // 等待云台稳定（2秒，与Python版本保持一致）
                std::this_thread::sleep_for(std::chrono::seconds(2));
                
                // 等待图像接收
                std::unique_lock<std::mutex> lock(image_mutex);
                image_received = false; // 重置标志
                
                if (!image_cv.wait_for(lock, std::chrono::seconds(10), [&image_received]() { return image_received; })) {
                    // 超时未收到图像
                    RCLCPP_WARN(this->get_logger(), "Timeout waiting for front camera image at angle (%.1f, %.1f)", hori, vert);
                    continue; // 继续下一个角度
                }
                
                // 成功接收到图像
                if (!latest_image) {
                    RCLCPP_WARN(this->get_logger(), "Received null image message at angle (%.1f, %.1f)", hori, vert);
                    continue;
                }
                
                // 获取图像数据
                if (latest_image->data.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Received empty image data at angle (%.1f, %.1f)", hori, vert);
                    continue;
                }
                
                // 根据 Image4m 消息格式，计算实际图像数据大小
                // 使用 step * height 来确定实际数据大小，但不超过 DATA_MAX_SIZE
                size_t actual_data_size = std::min(
                    static_cast<size_t>(latest_image->step * latest_image->height),
                    static_cast<size_t>(custom_image_msg::msg::Image4m::DATA_MAX_SIZE)
                );
                
                // 将实际图像数据转换为vector
                std::vector<uint8_t> image_data(
                    latest_image->data.begin(), 
                    latest_image->data.begin() + actual_data_size
                );
                
                // 处理BGR格式的图像数据并转换为WebP
                std::vector<uint8_t> webp_data;
                // 检查图像编码是否为BGR格式
                /*latest_image->encoding == "bgr8" || latest_image->encoding == "bgr"*/
                if (1) {
                    RCLCPP_INFO(this->get_logger(), "Processing BGR image data and converting to WebP");
                    
                    // 推断BGR图像尺寸
                    int total_pixels = image_data.size() / 3;
                    std::vector<std::pair<int, int>> ratios = {{4, 3}, {16, 9}, {3, 2}, {1, 1}};
                    int width = 0, height = 0;
                    bool found = false;
                    
                    for (const auto& ratio : ratios) {
                        int w_ratio = ratio.first;
                        int h_ratio = ratio.second;
                        
                        double area = static_cast<double>(total_pixels);
                        double ratio_area = static_cast<double>(w_ratio * h_ratio);
                        double scale = std::sqrt(area / ratio_area);
                        
                        int w = static_cast<int>(std::round(scale * w_ratio));
                        int h = static_cast<int>(std::round(scale * h_ratio));
                        
                        if (w * h * 3 == static_cast<int>(image_data.size())) {
                            width = w;
                            height = h;
                            found = true;
                            break;
                        }
                    }
                    
                    if (!found) {
                        RCLCPP_WARN(this->get_logger(), "Could not determine image dimensions. Data size: %zu bytes", image_data.size());
                        continue;
                    }
                    
                    RCLCPP_INFO(this->get_logger(), "Decoded BGR image: %dx%d", width, height);
                    
                    // 使用libwebp将BGR数据编码为WebP
                    uint8_t* webp_output = nullptr;
                    size_t webp_size = 0;
                    
                    // 设置WebP编码参数
                    WebPConfig config;
                    if (!WebPConfigPreset(&config, WEBP_PRESET_PHOTO, 75.0f)) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to configure WebP preset");
                        continue;
                    }
                    
                    // 配置更多参数
                    config.method = 6;
                    config.quality = 75.0f;
                    config.target_size = 0;
                    config.target_PSNR = 0;
                    config.segments = 4;
                    config.sns_strength = 50;
                    config.filter_strength = 60;
                    config.filter_sharpness = 0;
                    config.filter_type = 1;
                    config.autofilter = 1;
                    config.alpha_compression = 1;
                    config.alpha_filtering = 1;
                    config.alpha_quality = 100;
                    config.pass = 1;
                    config.show_compressed = 0;
                    config.preprocessing = 4;
                    config.partitions = 3;
                    config.partition_limit = 0;
                    config.emulate_jpeg_size = 0;
                    config.thread_level = 1;
                    config.low_memory = 0;
                    config.near_lossless = 100;
                    config.exact = 0;
                    config.use_delta_palette = 0;
                    config.use_sharp_yuv = 1;
                    
                    // 检查配置是否有效
                    if (!WebPValidateConfig(&config)) {
                        RCLCPP_ERROR(this->get_logger(), "Invalid WebP configuration");
                        continue;
                    }
                    
                    // 创建WebP图片对象
                    WebPPicture pic;
                    if (!WebPPictureInit(&pic)) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to initialize WebP picture");
                        continue;
                    }
                    
                    pic.width = width;
                    pic.height = height;
                    pic.use_argb = 1;
                    
                    // 将BGR数据导入到WebP图片对象
                    if (!WebPPictureImportBGR(&pic, image_data.data(), width * 3)) {
                        WebPPictureFree(&pic);
                        RCLCPP_ERROR(this->get_logger(), "Failed to import BGR data to WebP picture");
                        continue;
                    }
                    
                    // 分配内存用于存储编码后的WebP数据
                    WebPMemoryWriter writer;
                    WebPMemoryWriterInit(&writer);
                    pic.writer = WebPMemoryWrite;
                    pic.custom_ptr = &writer;
                    
                    // 执行编码
                    int ok = WebPEncode(&config, &pic);
                    if (!ok) {
                        WebPMemoryWriterClear(&writer);
                        WebPPictureFree(&pic);
                        RCLCPP_ERROR(this->get_logger(), "Failed to encode WebP image");
                        continue;
                    }
                    
                    // 获取编码后的WebP数据
                    webp_size = writer.size;
                    webp_output = writer.mem;
                    
                    // 将WebP数据复制到webp_data向量中
                    webp_data.assign(webp_output, webp_output + webp_size);
                    
                    // 清理资源
                    WebPMemoryWriterClear(&writer);
                    WebPPictureFree(&pic);
                    
                    // 使用WebP数据而不是原始BGR数据
                    image_data = std::move(webp_data);
                }
                
                // 将图像数据编码为base64
                std::string base64_data = base64_encode(image_data);
                
                // 构造完整的data URL (使用WebP格式)
                std::string data_url = "data:image/webp;base64," + base64_data;
                images.push_back(data_url);
                // 保存图像到本地文件
                std::string filename = "robot_watch_image_" + std::to_string(image_index) + "_" +
                                       std::to_string(static_cast<int>(hori)) + "_" +
                                       std::to_string(static_cast<int>(vert)) + ".webp";
                saveImageToFile(image_data, filename, "webp");
                RCLCPP_INFO(this->get_logger(), "Saved image to local file: %s", filename.c_str());

                RCLCPP_INFO(this->get_logger(), "Captured image at angle (%.1f, %.1f), size: %zu bytes", hori, vert, image_data.size());
                image_index++; // 增加图像索引
                RCLCPP_INFO(this->get_logger(), "Captured image at angle (%.1f, %.1f), size: %zu bytes", hori, vert, image_data.size());
            }
            
            // 重置云台到默认位置 (0, 0)
            sendGimbleControl(0.0f, 0.0f);
            
            // 清理订阅者
            image_sub.reset();
            
            // 构建返回结果
            if (images.empty()) {
                MCPToolResult result;
                result.isError = true;
                result.content = "Failed to capture any images";
                promise->set_value(result);
                return;
            }
            
            // 构建JSON数组格式的结果
            json images_array = json::array();
            for (const auto& img : images) {
                images_array.push_back(img);
            }
            
            MCPToolResult result;
            result.isError = false;
            result.content = images_array.dump();
            promise->set_value(result);
            
        }
        catch (const std::exception &e) {
            MCPToolResult result;
            result.isError = true;
            result.content = "Error getting environment images: " + std::string(e.what());
            promise->set_value(result);
        } })
        .detach();

    return future;
}
// 实现云台控制方法
void ChatNode::sendGimbleControl(float hori, float vert)
{
    // auto gimble_publisher = this->create_publisher<GimbleControl>("/gimble_control", 10);
    auto gimble_publisher = this->create_publisher<serial_interfaces::msg::GimbleControl>("/gimble_control", 10);
    
    // auto msg = std::make_shared<GimbleControl>();
    auto msg = std::make_shared<serial_interfaces::msg::GimbleControl>();
    msg->set_level_angle = hori;
    msg->set_vertical_angle = vert;
    
    gimble_publisher->publish(*msg);
    RCLCPP_DEBUG(this->get_logger(), "Sent gimble control command: hori=%.1f, vert=%.1f", hori, vert);
}

/**
 * 解析语义映射文件
 * @param file 映射文件路径
 * @return 语义映射字典
 */
SemanticMapping ChatNode::parseSemanticMapping(const std::string &file)
{
    SemanticMapping result_dict;
    std::ifstream f(file);

    if (!f.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open semantic mapping file: %s", file.c_str());
        return result_dict;
    }

    std::string line;
    int line_number = 1;
    while (std::getline(f, line))
    {
        std::istringstream iss(line);
        std::string index_str, en_name, chinese_name;

        if (iss >> index_str >> en_name >> chinese_name)
        {
            try
            {
                int index = std::stoi(index_str);
                result_dict[index] = std::make_pair(chinese_name, en_name);
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(this->get_logger(), "Failed to parse line %d in semantic mapping file: %s", line_number, line.c_str());
            }
        }
        line_number++;
    }

    f.close();
    return result_dict;
}

/**
 * 解析语义地图文件
 * @param file 语义地图文件路径
 * @param mapping_file 映射文件路径
 * @return 语义结果和映射值的pair
 */
std::pair<SemanticResult, std::vector<std::pair<std::string, std::string>>> 
ChatNode::parseSemanticMap(const std::string &file, const std::string &mapping_file)
{
    SemanticMapping semantic_mapping = parseSemanticMapping(mapping_file);
    for(auto &item : semantic_mapping)
    {
        std::cout << item.first << ": " << item.second.first << " " << item.second.second << std::endl;
    }
    SemanticResult result;
    std::vector<std::pair<std::string, std::string>> semantic_values;

    std::ifstream f(file);
    if (!f.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot open semantic map file: %s", file.c_str());
        return std::make_pair(result, semantic_values);
    }

    std::string line;
    std::vector<std::string> lines;

    // 读取所有行
    while (std::getline(f, line))
    {
        lines.push_back(line);
    }
    f.close();

    // 跳过前5行和最后1行
    if (lines.size() < 6)
    {
        RCLCPP_ERROR(this->get_logger(), "Semantic map file has insufficient lines");
        return std::make_pair(result, semantic_values);
    }
    // for(int i = 0; i < lines.size(); i++)
    // {
    //     std::cout << lines[i] << std::endl;
    // }
    // 取第6行到Finish行之前的数据
    std::vector<std::string> content_lines;
    for (size_t i = 5; i < lines.size(); ++i)
    {
        if (lines[i].find("Finish") != std::string::npos)
        {
            break; // 遇到包含"Finish"的行时停止
        }
        content_lines.push_back(lines[i]);
    }
    std::cout << "content_lines.size(): " << content_lines.size() << std::endl;
    for(int i = 0; i < content_lines.size(); i++)
    {
        std::cout << content_lines[i] << std::endl;
    }
    const int item_length = 9;
    if (content_lines.size() % item_length != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Semantic map file format error: line count is not multiple of %d", item_length);
        return std::make_pair(result, semantic_values);
    }

    // 解析每个语义项
    for (size_t i = 0; i < content_lines.size() / item_length; ++i)
    {
        SemanticItem semantic_item;

        // 解析语义标签
        std::string semantic_line = content_lines[i * item_length + 0];
        std::istringstream semantic_stream(semantic_line);
        std::string temp, index_str;
        semantic_stream >> temp >> temp >> index_str;

        // 获取中英文名称映射
        try
        {
            int index = std::stoi(index_str);
            auto mapping_it = semantic_mapping.find(index);
            if (mapping_it != semantic_mapping.end())
            {
                std::string cn_name = mapping_it->second.first;
                std::string en_name = mapping_it->second.second;
                semantic_item.semantic_label = en_name; // 使用英文名

                // 实例标签
                std::string instance_line = content_lines[i * item_length + 1];
                std::istringstream instance_stream(instance_line);
                instance_stream >> temp >> semantic_item.instance_label;

                // 建议点坐标
                std::string point_line = content_lines[i * item_length + 4];
                std::istringstream point_stream(point_line);
                point_stream >> temp >> temp >> semantic_item.suggest_point_x >> semantic_item.suggest_point_y;

                // 方向
                std::string orient_line = content_lines[i * item_length + 5];
                std::istringstream orient_stream(orient_line);
                orient_stream >> temp >> temp >> semantic_item.orient;

                // 将结果同时以英文名和中文名为键存储
                result[en_name] = semantic_item;
                result[cn_name] = semantic_item;

                // 收集映射值
                semantic_values.push_back(std::make_pair(cn_name, en_name));
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to process semantic item at index %zu: %s. Data: %s", 
                i, e.what(), semantic_line.c_str());
        }
    }

    return std::make_pair(result, semantic_values);
}

// 实现获取家具列表的MCP工具函数
std::future<MCPToolResult> ChatNode::getFurnitures(const json& /*arguments*/)
{
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    std::cout << "getFurnitures: " << furniture_list_.size() << std::endl;
    try {
        // 构建返回的JSON数组
        json furniture_array = json::array();
        for (const auto& furniture : furniture_list_) {
            furniture_array.push_back(furniture.first);//加入中文名
            furniture_array.push_back(furniture.second);//加入英文名
        }
        
        MCPToolResult result;
        result.isError = false;
        result.content = furniture_array.dump();
        promise->set_value(result);
    }
    catch (const std::exception& e) {
        MCPToolResult result;
        result.isError = true;
        result.content = "Error getting furniture list: " + std::string(e.what());
        promise->set_value(result);
    }
    
    return promise->get_future();
}

// 修改 moveNearFurniture 函数
std::future<MCPToolResult> ChatNode::moveNearFurniture(const json &arguments)
{
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    // 创建一个线程来执行移动任务，确保不阻塞主线程
    // std::thread([this, promise, arguments]()
    // {

    try
    {
        // 获取家具名称参数
        std::string furniture_name = arguments.value("furniture_name", "");
        if (furniture_name.empty())
        {
            MCPToolResult result;
            result.isError = true;
            result.content = "Missing furniture_name argument";
            promise->set_value(result);
            return future;
            // return;
        }

        // 查找家具信息
        if (semantic_result.find(furniture_name) == semantic_result.end())
        {
            RCLCPP_INFO(this->get_logger(), "Furniture not found: ");
            MCPToolResult result;
            result.isError = true;
            result.content = "Furniture not found: " + furniture_name;
            promise->set_value(result);
            return future;
            // return;
        }

        SemanticItem furniture = semantic_result.at(furniture_name);

        std::cout << "============= furniture ===================" << std::endl;
        std::cout << "furniture: " << " " << furniture.suggest_point_x << " " << furniture.suggest_point_y << " "
                  << furniture.instance_label << " " << furniture.semantic_label << " " << furniture.orient << std::endl;

        // 创建 GoTarget goal
        // 注意：orientation 需要翻转180度以获得正确的机器人朝向
        auto goal = createGoTargetGoal(furniture.suggest_point_x, furniture.suggest_point_y, furniture.orient + 180.0f);

        // 检查action客户端是否可用
        if (!go_target_client_ || !go_target_client_->action_server_is_ready())
        {
            RCLCPP_INFO(this->get_logger(), "GoTarget action server is not available ");
            MCPToolResult result;
            result.isError = true;
            result.content = "GoTarget action server is not available";
            promise->set_value(result);
            return future;
            // return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "GoTarget action server is ready");
        }

        RCLCPP_INFO(this->get_logger(), "Sending GoTarget goal to move near furniture: %s", furniture_name.c_str());

        // 发送目标
        auto send_goal_options = rclcpp_action::Client<app_msgs::action::GoTarget>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [](rclcpp_action::ClientGoalHandle<app_msgs::action::GoTarget>::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_WARN(rclcpp::get_logger("ChatNode"), "GoTarget goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("ChatNode"), "GoTarget goal accepted by server");
            }
        };

        send_goal_options.result_callback =
            [promise, furniture_name](const rclcpp_action::ClientGoalHandle<app_msgs::action::GoTarget>::WrappedResult &result)
        {
            MCPToolResult tool_result;
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if (result.result->result == 1)
                { // 根据Python代码，1表示成功
                    tool_result.isError = false;
                    tool_result.content = "已成功移动到 " + furniture_name + " 附近。如果需要查看周围环境，请调用 robot_watch 工具。";
                }
                else
                {
                    tool_result.isError = true;
                    tool_result.content = "Failed to move near " + furniture_name +
                                          " (result code: " + std::to_string(result.result->result) + ")";
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                tool_result.isError = true;
                tool_result.content = "Move to " + furniture_name + " was aborted";
                break;
            case rclcpp_action::ResultCode::CANCELED:
                tool_result.isError = true;
                tool_result.content = "Move to " + furniture_name + " was canceled";
                break;
            default:
                tool_result.isError = true;
                tool_result.content = "Unknown result code for moving to " + furniture_name;
                break;
            }
            promise->set_value(tool_result);
        };

        send_goal_options.feedback_callback =
            [](rclcpp_action::ClientGoalHandle<app_msgs::action::GoTarget>::SharedPtr goal_handle,
               const std::shared_ptr<const app_msgs::action::GoTarget::Feedback> feedback)
        {
            RCLCPP_INFO(rclcpp::get_logger("ChatNode"), "GoTarget feedback: status=%d", feedback->status);
        };
        auto goal_handle = go_target_client_->async_send_goal(goal, send_goal_options);
        // // 等待目标完成，确保在线程结束前 promise 被设置
        // // 这里可以设置一个超时时间
        // if (goal_handle.wait_for(std::chrono::seconds(30)) == std::future_status::timeout)
        // {
        //     MCPToolResult result;
        //     result.isError = true;
        //     result.content = "Navigation timeout";
        //     promise->set_value(result);
        // }
        auto status = std::future_status::timeout;
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout_duration = std::chrono::seconds(30);
        while (status != std::future_status::ready && rclcpp::ok())
        {
            status = goal_handle.wait_for(std::chrono::milliseconds(100));

            // 检查是否超时
            if (std::chrono::steady_clock::now() - start_time > timeout_duration)
            {
                MCPToolResult result;
                result.isError = true;
                result.content = "Navigation timeout";
                promise->set_value(result);
                return future;
            }

            // 检查是否收到中断信号
            if (!rclcpp::ok() || stop_.load())
            {
                // 可以选择取消目标
                // go_target_client_->async_cancel_goal(goal_handle);
                MCPToolResult result;
                result.isError = true;
                result.content = "Navigation cancelled by user";
                promise->set_value(result);
                return future;
            }
        }
    }
    catch (const std::exception &e)
    {
        MCPToolResult result;
        result.isError = true;
        result.content = "Error moving to furniture: " + std::string(e.what());
        promise->set_value(result);
    } //}).detach();

    return future;
}

// 确保 createGoTargetGoal 函数正确实现
app_msgs::action::GoTarget::Goal ChatNode::createGoTargetGoal(float x, float y, float orientation)
{
    app_msgs::action::GoTarget::Goal request;
    
    // 设置任务状态
    request.task_state = "start_task";
    
    // 设置运动类型为绝对运动(类型1)
    request.motion_type.type = 1;
    
    // 创建时间戳
    auto now = this->now();
    request.target_pose.header.stamp = now;
    request.target_pose.header.frame_id = "1";  // 使用地图坐标系
    
    // 设置目标位置
    request.target_pose.pose.position.x = static_cast<double>(x);
    request.target_pose.pose.position.y = static_cast<double>(y);
    request.target_pose.pose.position.z = 0.0;
    
    // 将欧拉角转换为四元数 (yaw为Z轴旋转，pitch和roll为0)
    tf2::Quaternion q;
    q.setRPY(0, 0, static_cast<double>(orientation) * M_PI / 180.0 + M_PI / 2);  // 将角度转换为弧度
    request.target_pose.pose.orientation.x = q.x();
    request.target_pose.pose.orientation.y = q.y();
    request.target_pose.pose.orientation.z = q.z();
    request.target_pose.pose.orientation.w = q.w();
    
    return request;
}

// 获取当前时间的回调函数
std::future<MCPToolResult> ChatNode::getCurrentTime(const json &arguments)
{
    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    MCPToolResult result;
    result.isError = false;
    
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    std::time_t time_t_now = std::chrono::system_clock::to_time_t(now);
    
    // 转换为本地时间
    std::tm* local_time = std::localtime(&time_t_now);
    
    // 格式化时间字符串
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%Y年%m月%d日 %H时%M分%S秒", local_time);
    
    result.content = std::string(buffer);
    promise->set_value(result);
    return future;
}

//外教口语模式
std::future<MCPToolResult> ChatNode::foreginTeaching(const json & /*arguments*/)
{
    std::cout << "============ foreginTeaching ==========" << std::endl;
    agent_type_ = agent_type::foreign_teach;
    initializeExternalRolePrompt();
    auto msg = std::make_shared<StringMsg>();
    msg->data = "[START]";
    stream_tts_publisher_->publish(*msg);
    
    msg->data = "外教口语任务已开始，你可以开始和我用英语交流了";
    stream_tts_publisher_->publish(*msg);

    msg->data = "[DONE]";
    stream_tts_publisher_->publish(*msg);

    auto promise = std::make_shared<std::promise<MCPToolResult>>();
    auto future = promise->get_future();

    MCPToolResult result;
    result.isError = false;
    result.content = "外教口语已开启";
    promise->set_value(result);

    return future;
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ChatNode>());
    auto node = std::make_shared<ChatNode>();

    // // 等待线程结束
    // node->join();  // ✅ 调用 public 方法，而不是直接访问成员
    node->mainLoop(); // 启动主循环
    rclcpp::shutdown();
    return 0;
}