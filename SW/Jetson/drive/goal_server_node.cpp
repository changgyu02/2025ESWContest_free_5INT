#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "drive_pkg/action/navigate_uwb_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <thread>
#include <atomic>

using NavigateUwbPose = drive_pkg::action::NavigateUwbPose;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateUwbPose>;

class GoalServer : public rclcpp::Node {
public:
    GoalServer() : Node("goal_server_node") {
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<NavigateUwbPose>(
            this, "navigate_uwb_pose",
            std::bind(&GoalServer::handle_goal, this, _1, _2),
            std::bind(&GoalServer::handle_cancel, this, _1),
            std::bind(&GoalServer::handle_accepted, this, _1)
        );

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        drive_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/drive_done", 10);
        position_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/position_done", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GoalServer::odom_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "✅ Goal Action Server is up and running.");
    }

private:
    rclcpp_action::Server<NavigateUwbPose>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drive_done_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr position_done_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    nav_msgs::msg::Odometry current_odom_;
    std::atomic<bool> has_pose_{false};

    geometry_msgs::msg::PoseStamped goal_pose_;
    bool has_goal_ = false;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
        has_pose_ = true;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateUwbPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "📥 Received goal: x=%.2f, y=%.2f, yaw=%.2f deg",
                    goal->x, goal->y, goal->yaw_deg);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
        RCLCPP_WARN(this->get_logger(), "⚠️ Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
        std::thread{std::bind(&GoalServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<NavigateUwbPose::Feedback>();
        auto result = std::make_shared<NavigateUwbPose::Result>();

        goal_pose_.header.frame_id = "map";
        goal_pose_.header.stamp = this->now();
        goal_pose_.pose.position.x = goal->x;
        goal_pose_.pose.position.y = goal->y;

        tf2::Quaternion q;
        q.setRPY(0, 0, goal->yaw_deg * M_PI / 180.0);
        goal_pose_.pose.orientation = tf2::toMsg(q);
        has_goal_ = true;

        goal_pub_->publish(goal_pose_);
        RCLCPP_INFO(this->get_logger(), "📤 Published /goal_pose to planner_node");

        rclcpp::Rate rate(5);
        const double dist_thresh = 0.25;
        const double yaw_thresh_rad = 10.0 * M_PI / 180.0;
        int timeout_ticks = 300;
        bool position_done_published = false;

        while (rclcpp::ok() && !goal_handle->is_canceling() && timeout_ticks-- > 0) {
            if (!has_pose_) {
                rate.sleep();
                continue;
            }

            double dx = goal->x - current_odom_.pose.pose.position.x;
            double dy = goal->y - current_odom_.pose.pose.position.y;
            double dist = std::hypot(dx, dy);

            double yaw_cur = get_yaw_from_quat(current_odom_.pose.pose.orientation);
            double yaw_goal = goal->yaw_deg * M_PI / 180.0;
            double yaw_err = normalize_angle(yaw_goal - yaw_cur);
            double yaw_err_deg = yaw_err * 180.0 / M_PI;

            feedback->distance_remaining = dist;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "📐 Yaw error: %.2f deg, dist: %.2f m", yaw_err_deg, dist);

            // ✅ 위치 도달 시 /position_done 한 번만 발행 (기존 유지)
            if (!position_done_published && dist < dist_thresh) {
                std_msgs::msg::Bool pos_done;
                pos_done.data = true;
                position_done_pub_->publish(pos_done);
                position_done_published = true;
                RCLCPP_INFO(this->get_logger(), "📍 Position reached! Published /position_done=true");
            }

            // ✅ 드라이브 완료는 '방향만'으로 판단하되, 반드시 position_done 이후에만 성공 처리
            //    (좌표 dist 조건 제거, 순서 보장)
            if (position_done_published && std::abs(yaw_err) < yaw_thresh_rad) {
                result->success = true;
                result->message = "도착 완료";
                goal_handle->succeed(result);

                std_msgs::msg::Bool done_msg;
                done_msg.data = true;
                drive_done_pub_->publish(done_msg);
                RCLCPP_INFO(this->get_logger(), "🎯 Goal reached! Published /drive_done=true");

                goal_pose_ = geometry_msgs::msg::PoseStamped();
                has_goal_ = false;
                RCLCPP_INFO(this->get_logger(), "🧹 goal_pose 초기화됨");
                return;
            }

            rate.sleep();
        }

        // 실패 처리
        if (goal_handle->is_canceling()) {
            result->success = false;
            result->message = "Goal canceled";
            goal_handle->canceled(result);
        } else {
            result->success = false;
            result->message = "Timeout or unknown failure";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "⏰ Goal aborted due to timeout");
        }

        goal_pose_ = geometry_msgs::msg::PoseStamped();
        has_goal_ = false;
        RCLCPP_INFO(this->get_logger(), "🧹 goal_pose 초기화됨 (abort)");
    }

    double get_yaw_from_quat(const geometry_msgs::msg::Quaternion& q) {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalServer>());
    rclcpp::shutdown();
    return 0;
}
