//
// Created by mmz on 2026/4/6.
//

#ifndef IMU_SERIAL_SERIAL_BOOST_H
#define IMU_SERIAL_SERIAL_BOOST_H

#include <boost/asio.hpp>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace IMU {
    struct ImuFrame {
        uint8_t type;
        uint16_t data1; //先低8位后高8位
        uint16_t data2;
        uint16_t data3;
        uint16_t data4;

        ImuFrame()=default;
    };

    class serial_boost {
    private:
        rclcpp::Logger logger_;

        boost::asio::io_context &io_;
        boost::asio::serial_port port_;
        boost::asio::deadline_timer timer_;
        std::vector<uint8_t> receive_buffer_; // 接收缓冲区，用于拼接数据包
        std::vector<uint8_t> buffer_; // 临时缓冲区，用于异步读取

        const std::string &port_name;
        unsigned int baud_rate;
        bool connected;

        static std::string typeToString(uint8_t type);

        bool connect();
        bool read();
        void processBuffer();
    public:
        serial_boost(boost::asio::io_context &io, const std::string &port, unsigned int baud_rate);

        void run(); //TODO 合理的数据读取方式
        //TODO 合理的数据更新判断方式
    };
}


#endif //IMU_SERIAL_SERIAL_BOOST_H
