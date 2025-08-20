#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <atomic>

using std::placeholders::_1;
using std::placeholders::_2;

class GoHomeNode : public rclcpp::Node {
public:
  GoHomeNode() : Node("go_home_node") {
    // 파라미터 선언
    this->declare_parameter<double>("home_x", 0.5);
    this->declare_parameter<double>("home_y", 1.0);
    this->declare_parameter<double>("home_yaw_deg", 0.0);
    this->declare_parameter<double>("dist_threshold_m", 0.10);
    this->declare_parameter<double>("yaw_threshold_deg", 10.0);
    this->declare_parameter<std::string>("goal_frame_id", "map");
    this->declare_parameter<double>("timeout_sec", 60.0);  // 선택

    // 서비스 서버
    service_ = this->create_service<std_srvs::srv::Trigger>(
        "go_home",
        std::bind(&GoHomeNode::handle_request, this, _1, _2));

    // 퍼블리셔
    goal_pub_         = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    home_done_pub_    = this->create_publisher<std_msgs::msg::Bool>("/home_return_done", 10);
    position_done_pub_= this->create_publisher<std_msgs::msg::Bool>("/position_done", 10);

    // 오도메트리 구독자
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoHomeNode::odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "go_home_node 초기화 완료");
  }

private:
  // ROS I/F
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr home_done_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr position_done_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // 상태
  nav_msgs::msg::Odometry current_odom_;
  std::atomic<bool> has_pose_{false};

  bool waiting_for_home_ = false;
  bool position_done_published_ = false;
  bool home_published_ = false;

  // 목표와 임계값
  double home_x_{0.5};
  double home_y_{1.0};
  double home_yaw_rad_{0.0};
  double dist_thresh_{0.10};
  double yaw_thresh_rad_{10.0 * M_PI / 180.0};
  std::string goal_frame_id_{"map"};
  rclcpp::Time start_time_;
  double timeout_sec_{60.0};

  // 유틸
  static double get_yaw_from_quat(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
  }
  static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
    has_pose_ = true;

    if (!waiting_for_home_) return;

    // 타임아웃 체크(선택)
    if (timeout_sec_ > 0.0 && (this->now() - start_time_).seconds() > timeout_sec_) {
      RCLCPP_ERROR(this->get_logger(), "원점 복귀 타임아웃");
      reset_state();
      return;
    }

    // 1) 위치 오차 판단 → /position_done 1회 발행
    if (!position_done_published_) {
      const double dx = current_odom_.pose.pose.position.x - home_x_;
      const double dy = current_odom_.pose.pose.position.y - home_y_;
      const double dist = std::hypot(dx, dy);

      RCLCPP_DEBUG(this->get_logger(), "거리=%.3f m (임계=%.3f)", dist, dist_thresh_);

      if (dist < dist_thresh_) {
        std_msgs::msg::Bool msg; msg.data = true;
        position_done_pub_->publish(msg);
        position_done_published_ = true;
        RCLCPP_INFO(this->get_logger(), "/position_done=true 퍼블리시");
      } else {
        return; // 아직 위치 못 도달 → 방향 판단 이전 단계 유지
      }
    }

    // 2) 방향(Yaw) 오차 판단 → 최종 /home_return_done 발행
    const double yaw_cur  = get_yaw_from_quat(current_odom_.pose.pose.orientation);
    const double yaw_err  = normalize_angle(home_yaw_rad_ - yaw_cur);
    const double yaw_err_deg = yaw_err * 180.0 / M_PI;

    RCLCPP_DEBUG(this->get_logger(), "Yaw 오차=%.2f deg (임계=%.2f deg)",
                 yaw_err_deg, yaw_thresh_rad_ * 180.0 / M_PI);

    if (std::abs(yaw_err) < yaw_thresh_rad_) {
      if (!home_published_) {
        std_msgs::msg::Bool done_msg; done_msg.data = true;
        home_done_pub_->publish(done_msg);
        home_published_ = true;
        RCLCPP_INFO(this->get_logger(), "/home_return_done=true 퍼블리시 (거리→방향 순서 충족)");
      }
      reset_state(); // 완료 후 상태 초기화
    }
  }

  void handle_request(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;

    // 최신 파라미터 로드
    this->get_parameter("home_x", home_x_);
    this->get_parameter("home_y", home_y_);
    double home_yaw_deg; this->get_parameter("home_yaw_deg", home_yaw_deg);
    home_yaw_rad_ = home_yaw_deg * M_PI / 180.0;

    this->get_parameter("dist_threshold_m", dist_thresh_);
    double yaw_threshold_deg; this->get_parameter("yaw_threshold_deg", yaw_threshold_deg);
    yaw_thresh_rad_ = yaw_threshold_deg * M_PI / 180.0;

    this->get_parameter("goal_frame_id", goal_frame_id_);
    this->get_parameter("timeout_sec", timeout_sec_);

    // goal_pose 발행
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = goal_frame_id_;
    goal_msg.header.stamp = this->now();
    goal_msg.pose.position.x = home_x_;
    goal_msg.pose.position.y = home_y_;

    tf2::Quaternion q;
    q.setRPY(0, 0, home_yaw_rad_);
    goal_msg.pose.orientation = tf2::toMsg(q);

    goal_pub_->publish(goal_msg);
    RCLCPP_INFO(this->get_logger(), "/goal_pose 발행: (%.3f, %.3f, yaw=%.1f deg, frame=%s)",
                home_x_, home_y_, home_yaw_deg, goal_frame_id_.c_str());

    // 상태 초기화 후 대기 시작
    waiting_for_home_ = true;
    position_done_published_ = false;
    home_published_ = false;
    start_time_ = this->now();

    response->success = true;
    response->message = "원점 복귀 명령이 실행되었습니다.";
  }

  void reset_state() {
    waiting_for_home_ = false;
    position_done_published_ = false;
    home_published_ = false;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoHomeNode>());
  rclcpp::shutdown();
  return 0;
}
