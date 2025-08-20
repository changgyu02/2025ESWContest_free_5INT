#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <queue>
#include <cmath>
#include <limits>
#include <map>
#include <vector>

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : Node("planner_node") {
    // ===== 파라미터 =====
    map_yaml_path_          = this->declare_parameter<std::string>("map_yaml", "/home/changgyu/ros2_ws/src/map.yaml");
    robot_radius_m_         = this->declare_parameter<double>("robot_radius_m", 0.135);
    max_snap_radius_m_      = this->declare_parameter<double>("max_snap_radius_m", 0.5);   // 스냅 반경 [m]
    use_last_valid_pose_    = this->declare_parameter<bool>("use_last_valid_pose", true);
    pose_ok_streak_req_     = this->declare_parameter<int>("pose_ok_streak_req", 3);
    hold_last_path_on_fail_ = this->declare_parameter<bool>("hold_last_path_on_fail", true);
    retry_scales_           = this->declare_parameter<std::vector<double>>(
                                "retry_scales", std::vector<double>{1.0, 0.85, 0.7});

    // 다운샘플 파라미터
    downsample_interval_m_  = this->declare_parameter<double>("downsample_interval_m", 0.08); // 기본 간격
    angle_keep_deg_         = this->declare_parameter<double>("angle_keep_deg", 12.0);        // 꺾임 보존 임계각
    goal_dense_radius_m_    = this->declare_parameter<double>("goal_dense_radius_m", 0.35);   // 목표 근접 반경
    goal_interval_m_        = this->declare_parameter<double>("goal_interval_m", 0.07);       // 목표 근접 간격
    danger_distance_m_      = this->declare_parameter<double>("danger_distance_m", 0.25);     // 장애물 근접 판단 거리
    danger_interval_m_      = this->declare_parameter<double>("danger_interval_m", 0.07);     // 장애물 근접 간격

    load_map(map_yaml_path_);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&PlannerNode::goal_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PlannerNode::odom_callback, this, std::placeholders::_1));

    position_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/position_done", 10, std::bind(&PlannerNode::position_done_callback, this, std::placeholders::_1));

    path_pub_        = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    plan_failed_pub_ = this->create_publisher<std_msgs::msg::Bool>("/plan_failed", 10);

    RCLCPP_INFO(this->get_logger(), "Planner Node Initialized (adaptive downsampling enabled)");
  }

private:
  // ===== ROS IO =====
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr position_done_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr plan_failed_pub_;

  // ===== 상태 =====
  nav_msgs::msg::Odometry current_odom_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool has_goal_{false};
  bool has_pose_{false};
  bool stop_path_publish_{false}; // position_done 이후 빈 경로만 발행

  // 마지막 성공 경로(실패 시 유지)
  nav_msgs::msg::Path last_valid_path_;
  bool has_last_valid_path_{false};

  // 마지막 정상 포즈(맵 free)
  geometry_msgs::msg::PoseStamped last_valid_world_pose_;
  int pose_ok_streak_{0};
  bool have_last_valid_pose_{false};

  // ===== 맵 / 파라미터 =====
  std::string map_yaml_path_;
  cv::Mat base_map_gray_;    // 원본 그레이(0=벽, 255=자유)
  cv::Mat inflated_map_;     // 팽창 적용 이진맵(255=자유, 0=장애물)
  cv::Mat dist_to_obs_m_;    // 자유영역의 각 픽셀에 대해 "최근접 장애물까지의 거리[m]"
  double resolution_{0.02};
  std::vector<double> origin_; // [x, y, yaw]
  double robot_radius_m_{0.135};
  double max_snap_radius_m_{0.5};
  bool   use_last_valid_pose_{true};
  int    pose_ok_streak_req_{3};
  bool   hold_last_path_on_fail_{true};
  std::vector<double> retry_scales_;

  // 적응형 다운샘플 파라미터
  double downsample_interval_m_{0.08};
  double angle_keep_deg_{12.0};
  double goal_dense_radius_m_{0.35};
  double goal_interval_m_{0.07};
  double danger_distance_m_{0.25};
  double danger_interval_m_{0.07};

  // ===== 콜백 =====
  void position_done_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      stop_path_publish_ = true;
      RCLCPP_INFO(this->get_logger(), "Received /position_done -> publish empty path only");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
    has_pose_ = true;

    // 현재 포즈가 유효한지 검사 → last_valid_world_pose_ 갱신
    if (map_ready()) {
      cv::Point pix = world_to_pixel(current_odom_.pose.pose.position.x,
                                     current_odom_.pose.pose.position.y);
      if (in_bounds(pix) && is_free(pix)) {
        pose_ok_streak_++;
        if (pose_ok_streak_ >= pose_ok_streak_req_) {
          last_valid_world_pose_ = odom_to_poseStamped(current_odom_);
          have_last_valid_pose_ = true;
        }
      } else {
        pose_ok_streak_ = 0;
      }
    }

    try_plan();
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (std::isnan(msg->pose.position.x) || std::isnan(msg->pose.position.y)) {
      RCLCPP_WARN(this->get_logger(), "Invalid goal_pose: NaN coordinates -> skip planning");
      return;
    }
    if (msg->pose.position.x > 9000 || msg->pose.position.y > 9000) {
      RCLCPP_WARN(this->get_logger(), "Dummy goal_pose received -> skip planning");
      return;
    }
    goal_pose_ = *msg;
    has_goal_ = true;
    stop_path_publish_ = false; // 새 목표 수신 시 경로 발행 재개
    try_plan();
  }

  // ===== 맵 로드/팽창/거리장 =====
  void load_map(const std::string &yaml_path) {
    YAML::Node map_yaml = YAML::LoadFile(yaml_path);
    std::string pgm_path = map_yaml["image"].as<std::string>();
    if (pgm_path.find('/') != 0)
      pgm_path = yaml_path.substr(0, yaml_path.find_last_of('/')) + "/" + pgm_path;

    double yaml_res = map_yaml["resolution"].as<double>();
    origin_ = map_yaml["origin"].as<std::vector<double>>();
    base_map_gray_ = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);

    if (base_map_gray_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map image: %s", pgm_path.c_str());
      return;
    }

    if (std::fabs(yaml_res - 0.02) > 1e-6) {
      RCLCPP_WARN(this->get_logger(),
                  "Map YAML resolution is %.5f m/px (expected 0.02000). Proceeding with YAML value.", yaml_res);
    }
    resolution_ = yaml_res;

    RCLCPP_INFO(this->get_logger(), "Map loaded: %s (%d x %d), resolution=%.5f m/px",
                pgm_path.c_str(), base_map_gray_.cols, base_map_gray_.rows, resolution_);

    make_inflated_map(robot_radius_m_);
  }

  void make_inflated_map(double robot_radius_m) {
    // 원본을 이진화(>200 = free)
    cv::Mat bin;
    cv::threshold(base_map_gray_, bin, 200, 255, cv::THRESH_BINARY);

    // 팽창 반경 픽셀(erosion으로 free 축소 = 장애물 확장 효과)
    int r_px = std::max(0, static_cast<int>(std::round(robot_radius_m / resolution_)));
    if (r_px > 0) {
      int k = 2 * r_px + 1;
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
      cv::erode(bin, inflated_map_, kernel);
    } else {
      inflated_map_ = bin.clone();
    }

    // 거리장: 자유영역 각 픽셀의 최근접 장애물까지의 거리(미터)
    // distanceTransform 입력의 비영(=free) 픽셀에 대해 0 픽셀까지의 거리 반환
    cv::Mat dist_px;
    cv::distanceTransform(inflated_map_, dist_px, cv::DIST_L2, 3);
    dist_to_obs_m_ = dist_px * resolution_;

    RCLCPP_INFO(this->get_logger(),
                "Inflation done (radius %.3f m -> %d px); distance field computed.", robot_radius_m, r_px);
  }

  // ===== 좌표 변환/체크 =====
  inline bool map_ready() const { return !base_map_gray_.empty() && !inflated_map_.empty(); }

  inline bool in_bounds(const cv::Point &pt) const {
    return pt.x >= 0 && pt.x < inflated_map_.cols && pt.y >= 0 && pt.y < inflated_map_.rows;
  }

  inline bool is_free(const cv::Point &pt) const {
    return inflated_map_.at<uchar>(pt) > 200;
  }

  inline double obs_distance_m_at(const cv::Point& pt) const {
    if (!in_bounds(pt)) return 0.0;
    return dist_to_obs_m_.at<float>(pt);
  }

  cv::Point world_to_pixel(double x, double y) const {
    int px = static_cast<int>((x - origin_[0]) / resolution_);
    int py = static_cast<int>(inflated_map_.rows - (y - origin_[1]) / resolution_);
    return cv::Point(px, py);
  }

  geometry_msgs::msg::PoseStamped pixel_to_world_pose(int px, int py) const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();
    pose.pose.position.x = px * resolution_ + origin_[0];
    pose.pose.position.y = (inflated_map_.rows - py) * resolution_ + origin_[1];
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  geometry_msgs::msg::PoseStamped odom_to_poseStamped(const nav_msgs::msg::Odometry &odom) const {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.header.frame_id = "map"; // 경로 프레임에 맞춤(필요시 수정)
    ps.pose = odom.pose.pose;
    return ps;
  }

  // 주변 free 픽셀 탐색(스냅)
  bool find_nearest_free_pixel(const cv::Point &from, int max_r_px, cv::Point &out) const {
    if (!in_bounds(from)) return false;
    if (is_free(from)) { out = from; return true; }
    for (int r = 1; r <= max_r_px; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        int y = from.y + dy;
        int dx = r - std::abs(dy);
        cv::Point p1(from.x - dx, y);
        cv::Point p2(from.x + dx, y);
        if (in_bounds(p1) && is_free(p1)) { out = p1; return true; }
        if (in_bounds(p2) && is_free(p2)) { out = p2; return true; }
      }
    }
    return false;
  }

  // ===== 경로 생성 =====
  void try_plan() {
    if (!has_goal_ || !has_pose_ || !map_ready()) return;

    if (stop_path_publish_) {
      publish_empty_path();
      RCLCPP_INFO(this->get_logger(), "Published empty path due to /position_done");
      return;
    }

    // 시작 포즈 결정
    geometry_msgs::msg::PoseStamped start_world;
    cv::Point start_pix = world_to_pixel(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y);
    bool start_ok = in_bounds(start_pix) && is_free(start_pix);

    if (!start_ok && use_last_valid_pose_ && have_last_valid_pose_) {
      start_world = last_valid_world_pose_;
      start_pix = world_to_pixel(start_world.pose.position.x, start_world.pose.position.y);
      RCLCPP_WARN(this->get_logger(), "Current pose invalid -> using last_valid_pose for planning");
    } else {
      start_world = odom_to_poseStamped(current_odom_);
    }

    // 목표 포즈
    geometry_msgs::msg::PoseStamped goal_world = goal_pose_;
    cv::Point goal_pix = world_to_pixel(goal_world.pose.position.x, goal_world.pose.position.y);

    // 시작/목표 스냅
    int snap_r_px = std::max(1, (int)std::round(max_snap_radius_m_ / resolution_));
    bool start_snapped = false, goal_snapped = false;
    if (!in_bounds(start_pix) || !is_free(start_pix)) {
      cv::Point s2;
      if (find_nearest_free_pixel(start_pix, snap_r_px, s2)) {
        start_pix = s2;
        start_world = pixel_to_world_pose(s2.x, s2.y);
        start_snapped = true;
      }
    }
    if (!in_bounds(goal_pix) || !is_free(goal_pix)) {
      cv::Point g2;
      if (find_nearest_free_pixel(goal_pix, snap_r_px, g2)) {
        goal_pix = g2;
        geometry_msgs::msg::PoseStamped gpose = pixel_to_world_pose(g2.x, g2.y);
        gpose.pose.orientation = goal_world.pose.orientation; // yaw 유지
        goal_world = gpose;
        goal_snapped = true;
      }
    }
    if (start_snapped || goal_snapped) {
      RCLCPP_WARN(this->get_logger(),
                  "Snap applied (start:%d, goal:%d, radius=%.2f m -> %d px)",
                  (int)start_snapped, (int)goal_snapped, max_snap_radius_m_, snap_r_px);
    }

    // 팽창 축소 재시도
    bool success = false;
    nav_msgs::msg::Path best_path;

    for (size_t ri = 0; ri < retry_scales_.size(); ++ri) {
      double scale = retry_scales_[ri];
      make_inflated_map(robot_radius_m_ * scale);

      std::vector<cv::Point> px_path = a_star(start_pix, goal_pix);
      if (!px_path.empty()) {
        // 1) 직선 병합
        px_path = simplify_path(px_path);

        // 2) 월드 포즈로 변환
        std::vector<geometry_msgs::msg::PoseStamped> raw_path;
        raw_path.reserve(px_path.size());
        for (const auto &pt : px_path) raw_path.push_back(pixel_to_world_pose(pt.x, pt.y));

        // 3) 적응형 다운샘플
        std::vector<geometry_msgs::msg::PoseStamped> final_path =
            adaptive_downsample_path(raw_path,
                                     downsample_interval_m_,
                                     angle_keep_deg_,
                                     goal_dense_radius_m_,
                                     goal_interval_m_,
                                     danger_distance_m_,
                                     danger_interval_m_);

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp    = this->now();
        path_msg.poses           = final_path;

        best_path = path_msg;
        success = true;
        if (ri > 0) {
          RCLCPP_WARN(this->get_logger(), "Plan succeeded with reduced inflation scale %.2f", scale);
        }
        break;
      } else {
        RCLCPP_WARN(this->get_logger(), "Plan failed with inflation scale %.2f, trying next scale", scale);
      }
    }

    // 결과 처리
    if (success && !best_path.poses.empty()) {
      path_pub_->publish(best_path);
      last_valid_path_ = best_path;
      has_last_valid_path_ = true;

      std_msgs::msg::Bool ok; ok.data = false;
      plan_failed_pub_->publish(ok);

      RCLCPP_INFO(this->get_logger(), "Path generated. Final path size: %zu", best_path.poses.size());
    } else {
      std_msgs::msg::Bool fail; fail.data = true;
      plan_failed_pub_->publish(fail);

      if (hold_last_path_on_fail_ && has_last_valid_path_) {
        path_pub_->publish(last_valid_path_);
        RCLCPP_ERROR(this->get_logger(),
                     "Planning failed -> keep last valid path (hold_last_path_on_fail=true)");
      } else {
        publish_empty_path();
        RCLCPP_ERROR(this->get_logger(),
                     "Planning failed -> publish empty path. Higher-level recovery needed");
      }
    }
  }

  // ===== A* / 유틸 =====
  std::vector<cv::Point> a_star(cv::Point start, cv::Point goal) {
    int rows = inflated_map_.rows, cols = inflated_map_.cols;
    auto inb = [&](cv::Point pt){ return pt.x>=0 && pt.x<cols && pt.y>=0 && pt.y<rows; };
    auto free = [&](cv::Point pt){ return inflated_map_.at<uchar>(pt) > 200; };

    if (!inb(start) || !inb(goal) || !free(start) || !free(goal)) {
      return {};
    }

    // 8방향 이웃
    const std::vector<cv::Point> dirs = {
      { 1, 0}, { 0, 1}, {-1, 0}, { 0,-1},   // 직교 4방향
      { 1, 1}, {-1, 1}, {-1,-1}, { 1,-1}    // 대각 4방향
    };

    // 코너커팅 방지: 대각선 이동 시 양 옆 직교 칸도 free여야 함
    auto passable = [&](const cv::Point& a, const cv::Point& b){
      if (!inb(b) || !free(b)) return false;
      cv::Point d = b - a;
      if (d.x != 0 && d.y != 0) { // 대각선 이동이면
        cv::Point n1(a.x + d.x, a.y);
        cv::Point n2(a.x, a.y + d.y);
        if (!inb(n1) || !inb(n2) || !free(n1) || !free(n2)) return false;
      }
      return true;
    };

    // 옥타일 휴리스틱(8방향 격자에 적합)
    auto octile = [&](const cv::Point& p, const cv::Point& q){
      double dx = std::abs(p.x - q.x), dy = std::abs(p.y - q.y);
      return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
    };

    auto cmp = [](const std::pair<double, cv::Point>& a, const std::pair<double, cv::Point>& b){
      return a.first > b.first;
    };
    std::priority_queue<
      std::pair<double, cv::Point>,
      std::vector<std::pair<double, cv::Point>>,
      decltype(cmp)
    > open(cmp);

    auto lessPt = [](const cv::Point& a, const cv::Point& b){ return (a.y==b.y)?(a.x<b.x):(a.y<b.y); };
    std::map<cv::Point, cv::Point, decltype(lessPt)> parent(lessPt);
    std::map<cv::Point, double,     decltype(lessPt)> g(lessPt);

    open.push({0.0, start});
    parent[start] = start;
    g[start] = 0.0;

    while (!open.empty()) {
      cv::Point cur = open.top().second;
      open.pop();

      if (cur == goal) break;

      for (auto d : dirs) {
        cv::Point nxt = cur + d;
        if (!passable(cur, nxt)) continue;               // 코너커팅 방지 포함 통과성 검사
        double step_cost = std::hypot(d.x, d.y);         // 1 또는 sqrt(2)
        double cost = g[cur] + step_cost;
        if (!g.count(nxt) || cost < g[nxt]) {
          g[nxt] = cost;
          double pr = cost + octile(nxt, goal);          // 옥타일 휴리스틱
          open.push({pr, nxt});
          parent[nxt] = cur;
        }
      }
    }

    if (!parent.count(goal)) return {};
    std::vector<cv::Point> path;
    for (cv::Point c = goal; c != parent[c]; c = parent[c]) path.push_back(c);
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
  }

  // 동일 방향 직선 병합
  std::vector<cv::Point> simplify_path(const std::vector<cv::Point>& raw) {
    if (raw.size() <= 2) return raw;
    std::vector<cv::Point> out; out.reserve(raw.size());
    out.push_back(raw.front());
    cv::Point prev_dir = raw[1] - raw[0];
    for (size_t i=2;i<raw.size();++i){
      cv::Point dir = raw[i] - raw[i-1];
      if (dir != prev_dir){ out.push_back(raw[i-1]); prev_dir = dir; }
    }
    out.push_back(raw.back());
    return out;
  }

  // 적응형 다운샘플
  std::vector<geometry_msgs::msg::PoseStamped> adaptive_downsample_path(
      const std::vector<geometry_msgs::msg::PoseStamped>& raw,
      double base_interval_m,
      double angle_keep_deg,
      double goal_radius_m,
      double goal_interval_m,
      double danger_dist_m,
      double danger_interval_m)
  {
    std::vector<geometry_msgs::msg::PoseStamped> out;
    if (raw.empty()) return out;
    if (raw.size() == 1) { out.push_back(raw.front()); return out; }

    auto deg = [](double rad){ return rad * 180.0 / M_PI; };
    auto angle_between = [&](const cv::Point2d& a, const cv::Point2d& b){
      double na = std::hypot(a.x, a.y), nb = std::hypot(b.x, b.y);
      if (na < 1e-9 || nb < 1e-9) return 0.0;
      double cosv = std::max(-1.0, std::min(1.0, (a.x*b.x + a.y*b.y)/(na*nb)));
      return std::acos(cosv);
    };

    auto dist = [](const geometry_msgs::msg::Point& p, const geometry_msgs::msg::Point& q){
      return std::hypot(p.x - q.x, p.y - q.y);
    };

    const geometry_msgs::msg::Point goal = raw.back().pose.position;

    // 첫 점은 항상 유지
    out.push_back(raw.front());
    geometry_msgs::msg::Point last_kept = raw.front().pose.position;

    for (size_t i = 1; i+1 < raw.size(); ++i) {
      const auto &prev = raw[i-1].pose.position;
      const auto &curr = raw[i].pose.position;
      const auto &next = raw[i+1].pose.position;

      // 1) 각도 변화 보존
      cv::Point2d v1(curr.x - prev.x, curr.y - prev.y);
      cv::Point2d v2(next.x - curr.x, next.y - curr.y);
      double ang_deg = deg(angle_between(v1, v2));
      bool keep_for_angle = (ang_deg >= angle_keep_deg);

      // 2) 로컬 간격 결정
      double local_interval = base_interval_m;

      // 목표 근접 시 더 촘촘히
      double dist_to_goal = std::hypot(curr.x - goal.x, curr.y - goal.y);
      if (dist_to_goal <= goal_radius_m) {
        local_interval = std::min(local_interval, goal_interval_m);
      }

      // 장애물 근접 시 더 촘촘히 (거리장 이용)
      cv::Point curr_px = world_to_pixel(curr.x, curr.y);
      double obs_m = obs_distance_m_at(curr_px);
      if (obs_m <= danger_dist_m) {
        local_interval = std::min(local_interval, danger_interval_m);
      }

      // 3) 유지 여부 판단
      double seg = dist(last_kept, curr);
      if (keep_for_angle || seg >= local_interval) {
        out.push_back(raw[i]);
        last_kept = curr;
      }
    }

    // 마지막 점은 항상 유지
    out.push_back(raw.back());
    return out;
  }

  void publish_empty_path() {
    nav_msgs::msg::Path p;
    p.header.frame_id = "map";
    p.header.stamp = this->now();
    p.poses.clear();
    path_pub_->publish(p);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
