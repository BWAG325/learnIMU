cmake_minimum_required(VERSION 3.10)
project(imu_serial)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)
add_definitions(-O3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
        DIRECTORY src
)

include_directories(${Boost_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
        fmt::fmt
        TBB::tbb
        Boost::system
)

rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN IMU::ImuSerialNode
        EXECUTABLE imu_serial_node
)

ament_package()
