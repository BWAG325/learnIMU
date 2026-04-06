//
// Created by mmz on 2026/4/6.
//

#include "imu_serial/serial_boost.h"

#include <iostream>

namespace IMU {
    serial_boost::serial_boost(boost::asio::io_context &io, const std::string &port, unsigned int baud_rate)
        : logger_(rclcpp::get_logger("serial_boost")), io_(io), port_(io_, port), timer_(io_), port_name(port),
          baud_rate(baud_rate) {
        connected = connect();
        if (!connected) {
            RCLCPP_ERROR(logger_, "Failed to open %s reconnect", port.c_str());
        }
    }

    bool serial_boost::connect() {
        boost::system::error_code ec;

        if (port_.is_open()) {
            port_.close(ec);
        }
        port_.open(port_name, ec);
        if (ec) {
            RCLCPP_ERROR(logger_, "Failed to open %s: %s", port_name.c_str(), ec.message().c_str());
            return false;
        }

        using namespace boost::asio;
        port_.set_option(serial_port::baud_rate(baud_rate), ec);
        port_.set_option(serial_port::character_size(8), ec);
        port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one), ec);
        port_.set_option(serial_port::parity(serial_port::parity::none), ec);
        port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none), ec);

        if (ec) {
            RCLCPP_ERROR(logger_, "Failed to configure port: %s", ec.message().c_str());
            port_.close();
            return false;
        }

        RCLCPP_INFO(logger_, "Connected to %s at %d baud", port_name.c_str(), baud_rate);
        return true;
    }

    bool serial_boost::read() {
        if (!port_.is_open() || !connected) {
            RCLCPP_ERROR(logger_, "Failed to open %s, can't read data", port_name.c_str());
            return false;
        }
        timer_.expires_from_now(boost::posix_time::milliseconds(300));
        bool read_timeout = false;
        timer_.async_wait([&](auto ec) {
            if (!ec) {
                port_.cancel();
                read_timeout = true;
            }
        });
        buffer_.resize(1024);
        port_.async_read_some(boost::asio::buffer(buffer_, 1024),
                              [this](const boost::system::error_code &ec, std::size_t length) {
                                  if (!ec) {
                                      receive_buffer_.insert(receive_buffer_.end(), buffer_.data(),
                                                             buffer_.data() + length);
                                      processBuffer();
                                  } else {
                                      std::cerr << "Read error: " << ec.message() << std::endl;
                                  }
                              });

        return true;
    }

    void serial_boost::processBuffer() {
        while (receive_buffer_.size() >= 12) {
            // 至少要有帧头+type+data(8字节)+crc，共1+1+8+1=11
            // 查找帧头 0x55
            auto it = std::find(receive_buffer_.begin(), receive_buffer_.end(), 0x55);
            if (it == receive_buffer_.end()) {
                receive_buffer_.clear();
                break;
            }
            if (it != receive_buffer_.begin()) {
                receive_buffer_.erase(receive_buffer_.begin(), it);
            }
            const size_t frameSize = 11;
            if (receive_buffer_.size() < frameSize) {
                break; // 数据不足，等待更多数据
            }

            // 小端模式解析 (低字节在前，高字节在后)
            uint8_t type = receive_buffer_[1];
            uint16_t data1 = receive_buffer_[2] | (receive_buffer_[3] << 8);
            uint16_t data2 = receive_buffer_[4] | (receive_buffer_[5] << 8);
            uint16_t data3 = receive_buffer_[6] | (receive_buffer_[7] << 8);
            uint16_t data4 = receive_buffer_[8] | (receive_buffer_[9] << 8);
            uint8_t receivedCrc = receive_buffer_[10];

            // 计算校验和 取低8位
            uint8_t calculatedCrc = 0x55 + type;
            calculatedCrc += receive_buffer_[2];
            calculatedCrc += receive_buffer_[3];
            calculatedCrc += receive_buffer_[4];
            calculatedCrc += receive_buffer_[5];
            calculatedCrc += receive_buffer_[6];
            calculatedCrc += receive_buffer_[7];
            calculatedCrc += receive_buffer_[8];
            calculatedCrc += receive_buffer_[9];
            if (receivedCrc == calculatedCrc) {
                ImuFrame frame;
                frame.type = type;
                frame.data1 = data1;
                frame.data2 = data2;
                frame.data3 = data3;
                frame.data4 = data4;

                // 移除已处理的完整帧
                receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + frameSize);
            } else {
                RCLCPP_DEBUG(logger_, "CRC check failed");
                receive_buffer_.erase(receive_buffer_.begin()); // 丢弃开头的0x55
            }
        }
    }

    std::string serial_boost::typeToString(uint8_t type) {
        switch (type) {
            case 0x50: return "时间";
            case 0x51: return "加速度";
            case 0x52: return "角速度";
            case 0x53: return "角度";
            case 0x54: return "磁场";
            case 0x55: return "端口状态";
            case 0x56: return "气压高度";
            case 0x57: return "经纬度";
            case 0x58: return "地速";
            case 0x59: return "四元数";
            case 0x5a: return "GPS定位精度";
            case 0x5f: return "读取";
            default: return "unknown";
        }
    }
}

