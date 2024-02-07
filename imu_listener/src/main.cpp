#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuListener : public rclcpp::Node {
 public:
  ImuListener() : Node("imu_listener") {
    listener_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&ImuListener::ImuCallback, this, std::placeholders::_1));
  }

 private:
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "Received IMU Data:\n- Orientation: [x=%f, y=%f, z=%f, w=%f]\n- Angular Velocity: [x=%f, y=%f, z=%f]\n- Linear Acceleration: [x=%f, y=%f, z=%f]",
                msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
                msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
                msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    // Additional message processing can be done here.
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr listener_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuListener>());
  rclcpp::shutdown();
  return 0;
}
