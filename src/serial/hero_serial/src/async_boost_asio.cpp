//  还未测试，占个坑
// 异步串口通信类
// 使用boost::asio库实现
// 支持异步读写操作
// 使用线程池管理I/O操作
// 提供缓冲区管理功能
// 支持多种串口参数配置

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <mutex>
#include <queue>
#include <vector>

class AsyncSerialPort {
   public:
    AsyncSerialPort(const std::string& port, unsigned int baud_rate = 115200,
                    size_t read_buffer_size = 1024)
        : io_context_(),
          serial_port_(io_context_),
          read_buffer_(read_buffer_size) {
        Open(port, baud_rate);
        StartIoThread();
    }

    ~AsyncSerialPort() { Close(); }

    // 异步写接口
    void WriteAsync(const std::vector<uint8_t>& data) {
        {
            std::lock_guard<std::mutex> lock(write_mutex_);
            write_queue_.push(data);
        }

        if (!is_writing_) {
            StartWrite();
        }
    }

    // 获取已接收数据（清空缓冲区）
    std::vector<uint8_t> ReadBufferedData() {
        std::lock_guard<std::mutex> lock(receive_mutex_);
        std::vector<uint8_t> data;
        data.swap(receive_buffer_);
        return data;
    }

   private:
    void Open(const std::string& port, unsigned int baud_rate) {
        try {
            serial_port_.open(port);
            serial_port_.set_option(
                boost::asio::serial_port_base::baud_rate(baud_rate));
            serial_port_.set_option(
                boost::asio::serial_port_base::character_size(8));
            serial_port_.set_option(boost::asio::serial_port_base::stop_bits(
                boost::asio::serial_port_base::stop_bits::one));
            serial_port_.set_option(boost::asio::serial_port_base::parity(
                boost::asio::serial_port_base::parity::none));
            StartRead();
        } catch (const boost::system::system_error& e) {
            throw std::runtime_error("Serial open failed: " +
                                     std::string(e.what()));
        }
    }

    void Close() {
        io_context_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
        boost::system::error_code ec;
        serial_port_.close(ec);
    }

    void StartIoThread() {
        io_thread_ = std::thread([this]() { io_context_.run(); });
    }

    void StartRead() {
        serial_port_.async_read_some(
            boost::asio::buffer(read_buffer_),
            boost::bind(&AsyncSerialPort::HandleRead, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    void HandleRead(const boost::system::error_code& ec,
                    size_t bytes_transferred) {
        if (!ec && bytes_transferred > 0) {
            std::lock_guard<std::mutex> lock(receive_mutex_);
            receive_buffer_.insert(receive_buffer_.end(), read_buffer_.begin(),
                                   read_buffer_.begin() + bytes_transferred);
        }
        StartRead();  // 持续读取
    }

    void StartWrite() {
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (write_queue_.empty()) {
            is_writing_ = false;
            return;
        }

        is_writing_ = true;
        auto data = write_queue_.front();
        write_queue_.pop();

        boost::asio::async_write(
            serial_port_, boost::asio::buffer(data),
            boost::bind(&AsyncSerialPort::HandleWrite, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    void HandleWrite(const boost::system::error_code& ec, size_t) {
        if (!ec) {
            StartWrite();  // 继续写入队列数据
        } else {
            std::lock_guard<std::mutex> lock(write_mutex_);
            is_writing_ = false;
        }
    }

    // IO成员
    boost::asio::io_context io_context_;
    boost::asio::serial_port serial_port_;
    std::thread io_thread_;

    // 接收相关
    std::vector<uint8_t> read_buffer_;
    std::vector<uint8_t> receive_buffer_;
    std::mutex receive_mutex_;

    // 发送相关
    std::mutex write_mutex_;
    std::queue<std::vector<uint8_t>> write_queue_;
    bool is_writing_ = false;
};

// 使用案例
// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// class SerialNode : public rclcpp::Node
// {
// public:
//     SerialNode() : Node("serial_node")
//     {
//         // 初始化串口
//         serial_ = std::make_unique<AsyncSerialPort>("/dev/ttyUSB0", 115200);

//         // 创建发布者
//         publisher_ =
//         this->create_publisher<std_msgs::msg::String>("rx_topic", 10);

//         // 创建定时器（100ms周期）
//         timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),
//             std::bind(&SerialNode::TimerCallback, this));
//     }

// private:
//     void TimerCallback()
//     {
//         // 示例：周期性发送数据
//         SendPeriodicData();

//         // 处理接收数据
//         ProcessReceivedData();
//     }

//     void SendPeriodicData()
//     {
//         static uint8_t counter = 0;
//         std::vector<uint8_t> data = {'S', 'E', 'N', 'D', counter++};
//         serial_->WriteAsync(data);
//     }

//     void ProcessReceivedData()
//     {
//         auto received = serial_->ReadBufferedData();
//         if (!received.empty()) {
//             std::string msg(received.begin(), received.end());

//             // 发布到ROS话题
//             auto ros_msg = std_msgs::msg::String();
//             ros_msg.data = msg;
//             publisher_->publish(ros_msg);

//             RCLCPP_INFO(this->get_logger(), "Received %zu bytes",
//             received.size());
//         }
//     }

//     std::unique_ptr<AsyncSerialPort> serial_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char* argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SerialNode>());
//     rclcpp::shutdown();
//     return 0;
// }