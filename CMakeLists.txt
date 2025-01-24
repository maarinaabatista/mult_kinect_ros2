cmake_minimum_required(VERSION 3.5)
project(mult_kinect_ros2)


## Dependências do ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

## OpenCV
find_package(OpenCV REQUIRED)

## libusb
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

## libfreenect
find_library(FREENECT_LIB NAMES freenect REQUIRED)
include_directories(/usr/include/libfreenect)

# Incluir diretórios corretamente
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}  
  ${LIBUSB_INCLUDE_DIRS}
  /usr/include/opencv4
  /usr/include/libusb-1.0
)

# Criar a biblioteca do driver Kinect
add_library(kinect_driver src/kinect_driver.cpp)
target_link_libraries(kinect_driver ${OpenCV_LIBS} ${LIBUSB_LIBRARIES} ${FREENECT_LIB})

# Criar o nó principal do ROS2
add_executable(mult_kinect_ros2_node src/mult_kinect_ros2_node.cpp)
target_link_libraries(mult_kinect_ros2_node kinect_driver ${OpenCV_LIBS} ${LIBUSB_LIBRARIES} ${FREENECT_LIB})
ament_target_dependencies(mult_kinect_ros2_node rclcpp sensor_msgs cv_bridge OpenCV image_transport)

# Instalar o executável 
install(TARGETS mult_kinect_ros2_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

