#ifndef MULT_KINECT_ROS2_NODE_HPP_
#define MULT_KINECT_ROS2_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "mult_kinect_ros2/kinect_driver.hpp"

class MultKinectROS2Node : public rclcpp::Node {
public:
    MultKinectROS2Node();

private:
    void captureAndPublish();  // Método para capturar e publicar imagens no ROS2
    rclcpp::TimerBase::SharedPtr timer_;  // Timer para capturar imagens periodicamente
    std::vector<image_transport::Publisher> image_publishers_;  // Publicadores para cada Kinect
    KinectManager kinect_manager_;  // Gerenciador dos dispositivos Kinect
};

#endif  // MULT_KINECT_ROS2_NODE_HPP_
