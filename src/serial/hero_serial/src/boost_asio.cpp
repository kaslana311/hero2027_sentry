// 同步串口通信类 
// 使用boost::asio库实现  
// 支持同步读写操作   
// 使用中
#include "boost_asio.hpp"

#include <fcntl.h>

BoostAsio::BoostAsio() : serial_port(io_context) {}

bool BoostAsio::Init(std::string port_name,uint64_t baud_rate) {
  // 初始化串口
  port_name_ = port_name;
  baud_rate_ = baud_rate;

  // 确保串口已关闭
  if (serial_port.is_open()) {
    serial_port.close(ec);
    if (ec) {
      std::cerr << "关闭串口错误: " << ec.message() << std::endl;
    }
  }

  serial_port.open(port_name, ec);

  if (ec) {
    std::cerr << "串口打开异常,Error: " << ec.message() << std::endl;
    return false;
  }
  std::cout << "串口打开成功" << std::endl;
  // 设置串口参数
  serial_port.set_option(boost::asio::serial_port::baud_rate(baud_rate_));
  // 每个字符由八个位组成
  serial_port.set_option(boost::asio::serial_port::character_size(8));
  // 设置控制流
  serial_port.set_option(boost::asio::serial_port::flow_control(
      boost::asio::serial_port::flow_control::none));
  // 无校验
  serial_port.set_option(
      boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  // 一个停止位
  serial_port.set_option(boost::asio::serial_port::stop_bits(
      boost::asio::serial_port::stop_bits::one));
  // 超时时间
  // serial_port.set_option(boost::asio::serial_port::timeout(boost::posix_time::seconds(1)));
  return true;
}

bool BoostAsio::Reinit() {
  Close();                  // 确保关闭
  return Init(port_name_,baud_rate_);  // 重新初始化
}

bool BoostAsio::Write(const boost::asio::mutable_buffer &buffer) {
  if (!serial_port.is_open() && !Init(port_name_,baud_rate_)) {
    error_count_++;
    return false;
  }
  size_t bytes_written = serial_port.write_some(buffer, ec);
  if (ec) {
    std::cerr << "写入错误: " << ec.message() << std::endl;
    if (++error_count_ >= max_error_count_) {
      std::cerr << "错误次数超限，尝试重启串口..." << std::endl;
      if (Reinit()) {
        error_count_ = 0;  // 重启成功，重置计数器
      } else {
        error_count_ = max_error_count_;  // 防止计数器溢出
      }
    }
    return false;
  }

  error_count_ = 0;  // 成功写入，重置计数器
  return true;
}


bool BoostAsio::Read() {
  if (!serial_port.is_open() && !Init(port_name_,baud_rate_)) {
    error_count_++;
    return false;
  }                                                                                                                                                                   

  uint8_t temp_buffer[256];
  size_t bytes_read =
      serial_port.read_some(boost::asio::buffer(temp_buffer), ec);

  if (ec) {
    std::cerr << "读取错误: " << ec.message() << std::endl;
    if (++error_count_ >= max_error_count_) {
      std::cerr << "错误次数超限，尝试重启串口..." << std::endl;
      if (Reinit()) {
        error_count_ = 0;
      } else {
        error_count_ = max_error_count_;
      }
    }
    return false;
  }

  read_buffer.insert(read_buffer.end(), temp_buffer, temp_buffer + bytes_read);
  error_count_ = 0;  // 成功读取，重置计数器
  return true;
}
void BoostAsio::Close() {
  if (serial_port.is_open()) {
    serial_port.close(ec);
    if (ec) {
      std::cerr << "关闭串口错误: " << ec.message() << std::endl;
    }
  }
}

BoostAsio::~BoostAsio() { Close(); }