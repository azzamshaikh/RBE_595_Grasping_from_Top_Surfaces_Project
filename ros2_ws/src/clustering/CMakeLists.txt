cmake_minimum_required(VERSION 3.8)
project(clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_py REQUIRED)
find_package(tf2_eigen REQUIRED)  
find_package(clustering_itf REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  include
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(clustering src/clustering.cpp)
target_link_libraries(clustering ${PCL_LIBRARIES} ${tf2_ros_LIBRARIES} ${tf2_eigen_LIBRARIES})

ament_target_dependencies(
  clustering
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen  
  PCL
  pcl_conversions
  clustering_itf
)

# Install the executables
install(TARGETS
  clustering
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()