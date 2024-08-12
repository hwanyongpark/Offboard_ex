cmake_minimum_required(VERSION 3.8)
project(offboard_ex)

# 컴파일 옵션 설정
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)

# 실행 파일 추가 및 의존성 연결
add_executable(offboard_node src/offboard_node.cpp)
ament_target_dependencies(offboard_node rclcpp px4_msgs px4_ros_com)

# include 디렉토리 설정
target_include_directories(offboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# C++17 표준 요구
target_compile_features(offboard_node PUBLIC c_std_99 cxx_std_17)

# 실행 파일 설치
install(TARGETS offboard_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
        launch
        DESTINATION share/${PROJECT_NAME}/
)

# ament 패키지 설정
ament_package()
