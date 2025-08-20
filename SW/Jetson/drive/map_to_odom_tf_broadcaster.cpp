#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>

using namespace std::chrono_literals;

class MapToOdomBroadcaster : public rclcpp::Node {
public:
  MapToOdomBroadcaster() : Node("map_to_odom_tf_broadcaster") {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 주기적으로 TF 퍼블리시
    timer_ = this->create_wall_timer(100ms, std::bind(&MapToOdomBroadcaster::broadcast_tf, this));
  }

private:
  void broadcast_tf() {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "odom";

    // 필요 시 UWB offset을 여기에 설정
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapToOdomBroadcaster>());
  rclcpp::shutdown();
  return 0;
}