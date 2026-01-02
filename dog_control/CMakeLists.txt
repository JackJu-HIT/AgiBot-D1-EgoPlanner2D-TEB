cmake_minimum_required(VERSION 3.8)
project(dog_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

# 识别 x86_64 架构（兼容不同系统的返回值）
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(ARCH "x86_64")
# 识别 arm64/aarch64 架构
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ARCH "aarch64")
else()
    message(FATAL_ERROR "不支持的架构: ${CMAKE_HOST_SYSTEM_PROCESSOR}，请检查 lib 目录下是否有对应架构的库")
endif()

message(STATUS "检测到系统架构: ${ARCH}")


# 2. 设置SDK路径变量
set(SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
# --------------------------
# 链接对应架构的动态库
set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH}")
message(STATUS "将链接的库路径: ${LIB_PATH}")

find_library(MC_SDK_LIB 
    NAMES "mc_sdk_zsl_1_${ARCH}"  # 库名（不含前缀lib和后缀.so）
    PATHS ${LIB_PATH}
    NO_DEFAULT_PATH  # 只在指定路径查找，避免找到其他版本
    REQUIRED
)


# ==========================================
# 3. 编译 Driver Node (接收指令节点)
# ==========================================
add_executable(dog_driver_node src/dog_driver_node.cpp)

# 包含头文件路径
target_include_directories(dog_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ROS依赖
ament_target_dependencies(dog_driver_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs           
  tf2                
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

target_link_libraries(dog_driver_node   ${MC_SDK_LIB})

# ==========================================
# 4. 编译 Keyboard Node (键盘节点)
# ==========================================
add_executable(dog_keyboard_node src/dog_keyboard_node.cpp)

ament_target_dependencies(dog_keyboard_node
  rclcpp
  geometry_msgs
  std_msgs
)

# ==========================================
# 5. 安装规则
# ==========================================
install(TARGETS
  dog_driver_node
  dog_keyboard_node
  DESTINATION lib/${PROJECT_NAME}
)

# 将库文件安装到 share 目录，以防运行时需要动态加载
install(DIRECTORY lib/
  DESTINATION share/${PROJECT_NAME}/lib
)

ament_package()