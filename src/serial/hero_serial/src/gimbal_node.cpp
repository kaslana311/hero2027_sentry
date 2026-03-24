#include "gimbal_node.hpp"

static GimbalNode* g_gimbal_node_instance = nullptr;

void CV_to_Nav_Massage_Global_Callback(void* data) {
    if (g_gimbal_node_instance) {
        g_gimbal_node_instance->IsAimingDataCallback(data);
    }
}

void GimbalNode::IsAimingDataCallback(void* data) {
    std_msgs::msg::Bool msg;
    msg.data = cv_to_nav_massage_data.is_aiming;
    is_aiming_pub_->publish(msg);
}

GimbalNode::GimbalNode() : Node("gimbal_node") {
    g_gimbal_node_instance = this;
    device = new LinuxDevice();

    // 初始化串口
    if (!boost_asio_.Init("/dev/gimbal", 115200)) {
        RCLCPP_ERROR(this->get_logger(), "与视觉电脑通信串口初始化失败");
        rclcpp::shutdown();  // 结束程序
        return;
    }

    auto native_handle = boost_asio_.serial_port.lowest_layer().native_handle();
    serial_dev_ = (rt_device_t)(intptr_t)native_handle;

    RCLCPP_INFO(this->get_logger(),
                "与视觉电脑通信串口初始化成功，文件描述符: %d",
                static_cast<int>(native_handle));

    is_aiming_pub_ =
        this->create_publisher<std_msgs::msg::Bool>("is_aiming", 10);

    // 注册回调
    Attach_Receive_callback(
        CMD_ID_CV_to_Nav_Massage, sizeof(CV_to_Nav_Massage_t),
        (void*)&cv_to_nav_massage_data, CV_to_Nav_Massage_Global_Callback);

    thread_running_ = true;
    receive_thread_ = std::thread(&GimbalNode::ReceiveThreadFunc, this);
    RCLCPP_INFO(this->get_logger(), "云台串口接收线程已启动");
}

GimbalNode::~GimbalNode() {
    thread_running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    delete device;
}

void GimbalNode::ReceiveThreadFunc() {
    RCLCPP_INFO(this->get_logger(), "接收线程开始运行");

    while (rclcpp::ok() && thread_running_.load()) {
        // 阻塞式读取串口数据（数据来了立刻返回）
        if (boost_asio_.Read()) {
            const auto& data = boost_asio_.GetReadBuffer();

            if (!data.empty()) {
                // 尝试解析数据
                if (!Receive_Handle(const_cast<unsigned char*>(data.data()),
                                    static_cast<unsigned int>(data.size()))) {
                    // 解析失败时不打印日志，避免刷屏（可能是数据不完整）
                    // 如果需要调试，可以取消注释下面的行
                    RCLCPP_WARN(this->get_logger(), "数据解析失败，数据长度:%zu", data.size());
                }
                // 清空缓冲区，防止数据堆积
                // 注意：这是临时方案，未来应实现环形缓冲区和粘包处理
                boost_asio_.Consume(data.size());
            }
        } else {
            // 读取失败时，短暂休眠避免 CPU 空转
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    RCLCPP_INFO(this->get_logger(), "接收线程退出");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
