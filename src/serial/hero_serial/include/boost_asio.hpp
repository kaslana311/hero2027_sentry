
// 同步串口通信类 
// 使用boost::asio库实现  
// 支持同步读写操作   
// 使用中

#pragma once
#include <boost/asio.hpp>
#include <cstdint>
#include <iostream>
// #include <future>
// #include <thread>

class BoostAsio {
 public:
  BoostAsio();
  ~BoostAsio();
  bool Init(std::string port_name,uint64_t baud_rate);
  bool Write(const boost::asio::mutable_buffer& buffer);
  bool Read();
  void Close();
  bool Reinit();  // 重启串口

  const std::vector<uint8_t>& GetReadBuffer() const { return read_buffer; }
  void Consume(size_t n) {
    if (n <= read_buffer.size()) {
      read_buffer.erase(read_buffer.begin(), read_buffer.begin() + n);
    }
  }

//  private:
  boost::asio::io_context io_context;
  boost::asio::serial_port serial_port;
  boost::system::error_code ec;
  std::vector<uint8_t> read_buffer;  // 接收数据缓冲区
  std::string port_name_;                 // 保存串口名称用于重启
  uint64_t baud_rate_;                    // 保存波特率用于重启
  int error_count_;                       // 连续错误计数器
  static const int max_error_count_ = 3;  // 最大允许错误次数
};