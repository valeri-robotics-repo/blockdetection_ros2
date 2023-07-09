cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# System Dependancies
find_package(Eigen3 REQUIRED)

set(CMAKE_FIND_DEBUG_MODE 1)

find_package(Open3D HINTS ~/Open3d/lib/cmake/)

find_package(DetectBlocks REQUIRED)


find_package(tf2 REQUIRED )
find_package(tf2_ros REQUIRED )
include_directories(~Open3D/include)
include_directories(/home/valerie/Open3d_example/sharedlib/include)

add_executable(listener src/subscriber_member_function.cpp src/open3d_ros.cpp)
TARGET_LINK_LIBRARIES(listener detectblocks Open3D::Open3D )
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs tf2 tf2_ros )

install(TARGETS
#listener
  listener
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY  launch 
  DESTINATION share/${PROJECT_NAME}
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
