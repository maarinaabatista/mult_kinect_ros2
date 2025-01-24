#include "mult_kinect_ros2/mult_kinect_ros2_node.hpp"

MultKinectROS2Node::MultKinectROS2Node() : Node("mult_kinect_ros2_node") {
    if (!kinect_manager_.initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Falha ao inicializar os Kinects!");
        rclcpp::shutdown();
    }

    int num_devices = kinect_manager_.getNumDevices();
    for (int i = 0; i < num_devices; ++i) {
        image_publishers_.push_back(image_transport::create_publisher(this, "kinect" + std::to_string(i) + "/image"));
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(30),
        std::bind(&MultKinectROS2Node::captureAndPublish, this));
}

void MultKinectROS2Node::captureAndPublish() {
    int num_devices = kinect_manager_.getNumDevices();
    for (int i = 0; i < num_devices; ++i) {
        cv::Mat frame;
        if (kinect_manager_.captureFrame(i, frame)) {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            image_publishers_[i].publish(*msg);
            RCLCPP_INFO(this->get_logger(), "Publicando imagem do Kinect %d no tópico /kinect%d/image", i, i);
        } else {
            RCLCPP_WARN(this->get_logger(), "Falha ao capturar imagem do Kinect %d", i);
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultKinectROS2Node>());
    rclcpp::shutdown();
    return 0;
}
