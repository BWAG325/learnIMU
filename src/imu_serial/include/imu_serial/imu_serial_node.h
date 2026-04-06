//
// Created by mmz on 2026/4/5.
//

#ifndef IMU_SERIAL_IMU_SERIAL_H
#define IMU_SERIAL_IMU_SERIAL_H
#include <rclcpp/node.hpp>

namespace IMU {
    class ImuSerialNode : public rclcpp::Node {
    public:
        ImuSerialNode(const rclcpp::NodeOptions & options);
    private:

    };
}
#endif //IMU_SERIAL_IMU_SERIAL_H