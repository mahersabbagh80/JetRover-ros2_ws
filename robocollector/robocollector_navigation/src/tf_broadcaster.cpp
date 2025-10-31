#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class StaticTfBroadcaster : public rclcpp::Node {
public:
  StaticTfBroadcaster()
  : Node("static_tf_broadcaster") {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(500ms, [this]() {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->now();
      t.header.frame_id = "base_link";
      t.child_frame_id = "laser_frame";
      t.transform.translation.x = 0.2;  // adjust as needed
      t.transform.translation.y = 0.0;
      t.transform.translation.z = 0.15;
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = 0.0;
      t.transform.rotation.w = 1.0;
      broadcaster_->sendTransform(t);
    });
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
