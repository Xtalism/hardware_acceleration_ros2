// Receives the four wheel-encoder counts the DE0-Nano sends as UDP broadcasts
// (ethernet_tx_paket/fpga/sensors_udp, port 1026, 200 Hz) and
// publishes mecanum-wheel odometry:
//   odom          nav_msgs/Odometry (pose in odom, twist in base_link)
//   TF            odom -> base_link (publish_tf)
//   wheel_states  sensor_msgs/JointState: each wheel's angle (rad) and speed (rad/s),
//                 e.g. as feedback for the motor controller, or for
//                 robot_state_publisher to turn the wheels of a model in RViz2
//   wheel_markers visualization_msgs/MarkerArray: a label per wheel with its turns
//                 and RPM (wheel_markers parameter, 10 Hz)
//
// UDP payload, 30 bytes, big-endian: "EN", u32 sequence, u32 FPGA time (us)
// when the counts were latched, 4 x i32 counts (x4 decoding), 4 x u8 counters
// of undecodable steps.
//
// Wheels are front-left, front-right, rear-left, rear-right. encoder_for_wheel
// says which encoder input belongs to each wheel, and wheel_signs is +1 or -1 so
// that every wheel counts up while the robot drives forward. Mecanum rollers in
// the standard "X" layout (seen from above), REP 103 axes (x forward, y left):
//   vx = r/4 * ( w_fl + w_fr + w_rl + w_rr)
//   vy = r/4 * (-w_fl + w_fr + w_rl - w_rr)
//   wz = r/(4 (lx + ly)) * (-w_fl + w_fr - w_rl + w_rr)
// The pose integrates the count changes, so it has no quantisation drift; the
// speeds are measured over velocity_window seconds of FPGA time.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
constexpr size_t kPayloadSize = 30;

int64_t to_ns(const timespec & ts) {return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;}

int64_t now_ns()
{
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);   // same clock as the kernel receive timestamp
  return to_ns(ts);
}

uint32_t be32(const uint8_t * p) {return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | p[3];}

double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {return 0.0;}
  size_t i = std::min(v.size() - 1, size_t(p * double(v.size() - 1) + 0.5));
  std::nth_element(v.begin(), v.begin() + i, v.end());
  return v[i];
}

struct Sample
{
  double t;                       // FPGA time, s, unwrapped
  std::array<double, 4> angle;    // wheel angles, rad
};
}  // namespace

class UdpMecanumOdomNode : public rclcpp::Node
{
public:
  UdpMecanumOdomNode()
  : Node("udp_mecanum_odom")
  {
    port_ = declare_parameter("port", 1026);
    cpr_ = declare_parameter("counts_per_rev", 1320.0);
    radius_ = declare_parameter("wheel_radius", 0.040);
    lx_ = declare_parameter("half_length", 0.100);
    ly_ = declare_parameter("half_width", 0.100);
    auto enc = declare_parameter("encoder_for_wheel", std::vector<int64_t>{0, 1, 2, 3});
    auto signs = declare_parameter("wheel_signs", std::vector<int64_t>{1, 1, 1, 1});
    names_ = declare_parameter(
      "wheel_names", std::vector<std::string>{"front_left_wheel", "front_right_wheel",
        "rear_left_wheel", "rear_right_wheel"});
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    publish_tf_ = declare_parameter("publish_tf", true);
    window_ = declare_parameter("velocity_window", 0.05);
    bool markers = declare_parameter("wheel_markers", true);
    stats_period_ = declare_parameter("stats_period", 2.0);

    if (enc.size() != 4 || signs.size() != 4 || names_.size() != 4) {
      throw std::runtime_error("encoder_for_wheel, wheel_signs and wheel_names need 4 entries");
    }
    for (int i = 0; i < 4; ++i) {
      if (enc[i] < 0 || enc[i] > 3) {throw std::runtime_error("encoder_for_wheel entries must be 0..3");}
      enc_[i] = int(enc[i]);
      sign_[i] = signs[i] < 0 ? -1.0 : 1.0;
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("wheel_states", rclcpp::QoS(10));
    if (publish_tf_) {tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);}
    if (markers) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("wheel_markers", rclcpp::QoS(10));
    }

    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    int on = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(uint16_t(port_));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);   // also receives broadcasts
    if (fd_ < 0 || bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error("cannot bind UDP port " + std::to_string(port_) + ": " + strerror(errno));
    }

    RCLCPP_INFO(
      get_logger(), "listening on UDP port %ld: %.1f counts/turn, wheel radius %.3f m, "
      "half length %.3f m, half width %.3f m, encoders for FL FR RL RR %d %d %d %d, signs %+.0f %+.0f %+.0f %+.0f",
      port_, cpr_, radius_, lx_, ly_, enc_[0], enc_[1], enc_[2], enc_[3],
      sign_[0], sign_[1], sign_[2], sign_[3]);
    t_report_ = now_ns();
    thread_ = std::thread(&UdpMecanumOdomNode::rx_loop, this);
  }

  ~UdpMecanumOdomNode() override
  {
    running_ = false;
    if (thread_.joinable()) {thread_.join();}
    if (fd_ >= 0) {close(fd_);}
    RCLCPP_INFO(
      get_logger(), "total: %lu packets, %lu lost, %lu other packets; final pose x %.3f y %.3f yaw %.1f deg",
      total_, lost_, other_, x_, y_, th_ * 180.0 / M_PI);
  }

private:
  void rx_loop()
  {
    uint8_t buf[256];
    alignas(cmsghdr) char ctrl[CMSG_SPACE(sizeof(timespec))];

    while (running_ && rclcpp::ok()) {
      pollfd p{fd_, POLLIN, 0};
      if (poll(&p, 1, 200) > 0) {
        iovec iov{buf, sizeof(buf)};
        msghdr m{};
        m.msg_iov = &iov;
        m.msg_iovlen = 1;
        m.msg_control = ctrl;
        m.msg_controllen = sizeof(ctrl);
        ssize_t n = recvmsg(fd_, &m, 0);
        int64_t t_node = now_ns();
        int64_t t_kernel = t_node;
        for (cmsghdr * c = CMSG_FIRSTHDR(&m); c; c = CMSG_NXTHDR(&m, c)) {
          if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SCM_TIMESTAMPNS) {
            timespec ts;
            std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
            t_kernel = to_ns(ts);
          }
        }
        if (n > 0) {handle(buf, size_t(n), t_kernel, t_node);}
      }
      if (now_ns() - t_report_ >= int64_t(stats_period_ * 1e9)) {report();}
    }
  }

  void handle(const uint8_t * b, size_t n, int64_t t_kernel, int64_t t_node)
  {
    if (n != kPayloadSize || b[0] != 'E' || b[1] != 'N') {
      ++other_;
      return;
    }
    uint32_t seq = be32(b + 2);
    uint32_t t_us = be32(b + 6);
    std::array<int32_t, 4> counts;
    std::array<uint8_t, 4> errors;
    for (int i = 0; i < 4; ++i) {
      counts[i] = int32_t(be32(b + 10 + 4 * i));
      errors[i] = b[26 + i];
    }

    bool restart = !have_prev_ || seq - prev_seq_ == 0 || seq - prev_seq_ >= 1000000;
    if (restart) {
      if (have_prev_) {RCLCPP_WARN(get_logger(), "sequence jumped %u -> %u: FPGA restarted", prev_seq_, seq);}
      history_.clear();
      t_s_ = 0.0;
      for (int i = 0; i < 4; ++i) {err_seen_[i] = 0;}
    } else {
      lost_ += seq - prev_seq_ - 1;
      t_s_ += double(uint32_t(t_us - prev_t_us_)) / 1e6;
      // wheel angle changes: wrap-safe count differences, mapped to FL FR RL RR
      std::array<double, 4> d;
      for (int w = 0; w < 4; ++w) {
        int e = enc_[w];
        d[w] = sign_[w] * double(int32_t(uint32_t(counts[e]) - uint32_t(prev_counts_[e]))) * 2.0 * M_PI / cpr_;
        angle_[w] += d[w];
        err_seen_[w] += uint8_t(errors[e] - prev_errors_[e]);
      }
      double dx = radius_ / 4.0 * (d[0] + d[1] + d[2] + d[3]);
      double dy = radius_ / 4.0 * (-d[0] + d[1] + d[2] - d[3]);
      double dth = radius_ / (4.0 * (lx_ + ly_)) * (-d[0] + d[1] - d[2] + d[3]);
      double thm = th_ + dth / 2.0;
      x_ += dx * std::cos(thm) - dy * std::sin(thm);
      y_ += dx * std::sin(thm) + dy * std::cos(thm);
      th_ = std::atan2(std::sin(th_ + dth), std::cos(th_ + dth));
    }
    have_prev_ = true;
    prev_seq_ = seq;
    prev_t_us_ = t_us;
    prev_counts_ = counts;
    prev_errors_ = errors;
    ++total_;

    // wheel speeds over the last velocity_window seconds of FPGA time
    history_.push_back({t_s_, angle_});
    while (history_.size() > 2 && t_s_ - history_.front().t > window_) {history_.pop_front();}
    std::array<double, 4> w{0, 0, 0, 0};
    double span = t_s_ - history_.front().t;
    if (span > 0) {
      for (int i = 0; i < 4; ++i) {w[i] = (angle_[i] - history_.front().angle[i]) / span;}
    }
    double vx = radius_ / 4.0 * (w[0] + w[1] + w[2] + w[3]);
    double vy = radius_ / 4.0 * (-w[0] + w[1] + w[2] - w[3]);
    double wz = radius_ / (4.0 * (lx_ + ly_)) * (-w[0] + w[1] - w[2] + w[3]);
    wheel_speed_ = w;

    rclcpp::Time stamp(t_kernel, RCL_SYSTEM_TIME);
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = std::sin(th_ / 2.0);
    odom.pose.pose.orientation.w = std::cos(th_ / 2.0);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;
    // mecanum rollers slip, so be modest; z, roll and pitch are not measured
    const double pose_cov[6] = {1e-3, 1e-3, 1e6, 1e6, 1e6, 3e-3};
    const double twist_cov[6] = {1e-3, 1e-3, 1e6, 1e6, 1e6, 3e-3};
    for (int i = 0; i < 6; ++i) {
      odom.pose.covariance[i * 7] = pose_cov[i];
      odom.twist.covariance[i * 7] = twist_cov[i];
    }
    odom_pub_->publish(odom);

    if (tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header = odom.header;
      t.child_frame_id = base_frame_;
      t.transform.translation.x = x_;
      t.transform.translation.y = y_;
      t.transform.rotation = odom.pose.pose.orientation;
      tf_->sendTransform(t);
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = names_;
    js.position.assign(angle_.begin(), angle_.end());
    js.velocity.assign(w.begin(), w.end());
    joint_pub_->publish(js);
    if (marker_pub_ && ++marker_div_ >= 20) {       // 10 Hz at 200 packets/s
      marker_div_ = 0;
      publish_markers();
    }

    wakeup_.push_back((t_node - t_kernel) / 1e3);
    publish_.push_back((now_ns() - t_node) / 1e3);
  }

  void publish_markers()
  {
    static const char * label[4] = {"FL", "FR", "RL", "RR"};
    const double pos[4][2] = {{lx_, ly_}, {lx_, -ly_}, {-lx_, ly_}, {-lx_, -ly_}};
    visualization_msgs::msg::MarkerArray arr;
    for (int i = 0; i < 4; ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = base_frame_;               // stamp 0: drawn with the latest TF
      m.ns = "wheels";
      m.id = i;
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = pos[i][0];
      m.pose.position.y = pos[i][1] * 1.8;           // outside the wheel
      m.pose.position.z = radius_ + 0.04;
      m.pose.orientation.w = 1.0;
      m.scale.z = 0.022;                             // text height, m
      m.color.a = 1.0;
      m.color.r = 1.0;
      m.color.g = err_seen_[i] ? 0.3 : 1.0;          // red after decode errors
      m.color.b = err_seen_[i] ? 0.3 : 1.0;
      char text[96];
      std::snprintf(
        text, sizeof(text), "%s %+.2f turns\n%+.0f RPM%s", label[i], angle_[i] / (2.0 * M_PI),
        wheel_speed_[i] * 60.0 / (2.0 * M_PI), err_seen_[i] ? "\ndecode errors" : "");
      m.text = text;
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

  void report()
  {
    double period_s = (now_ns() - t_report_) / 1e9;
    t_report_ = now_ns();
    if (total_ == report_total_) {
      RCLCPP_WARN(
        get_logger(), "no encoder packets in %.1f s on UDP port %ld%s", period_s, port_,
        other_ ? " (other packets seen: is fpga/sensors_udp loaded?)" : "");
      return;
    }
    double rate = (total_ - report_total_) / period_s;
    report_total_ = total_;
    auto rpm = [this](int i) {return wheel_speed_[i] * 60.0 / (2.0 * M_PI);};
    RCLCPP_INFO(
      get_logger(),
      "%.0f Hz, lost %lu | wheel RPM FL %+.1f FR %+.1f RL %+.1f RR %+.1f, decode errors %u %u %u %u | "
      "pose x %+.3f y %+.3f m yaw %+.1f deg | node wake-up p50 %.0f max %.0f us, publish %.0f us",
      rate, lost_, rpm(0), rpm(1), rpm(2), rpm(3),
      err_seen_[0], err_seen_[1], err_seen_[2], err_seen_[3],
      x_, y_, th_ * 180.0 / M_PI,
      percentile(wakeup_, 0.5), percentile(wakeup_, 1.0), percentile(publish_, 0.5));
    wakeup_.clear();
    publish_.clear();
  }

  int64_t port_;
  double cpr_, radius_, lx_, ly_, window_, stats_period_;
  std::array<int, 4> enc_;
  std::array<double, 4> sign_;
  std::vector<std::string> names_;
  std::string odom_frame_, base_frame_;
  bool publish_tf_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  int marker_div_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;

  int fd_ = -1;
  std::thread thread_;
  std::atomic<bool> running_{true};

  bool have_prev_ = false;
  uint32_t prev_seq_ = 0, prev_t_us_ = 0;
  std::array<int32_t, 4> prev_counts_{};
  std::array<uint8_t, 4> prev_errors_{};
  std::array<unsigned, 4> err_seen_{};
  double t_s_ = 0.0;
  std::array<double, 4> angle_{};         // wheel angles FL FR RL RR, rad
  std::array<double, 4> wheel_speed_{};   // rad/s
  std::deque<Sample> history_;
  double x_ = 0.0, y_ = 0.0, th_ = 0.0;

  uint64_t total_ = 0, lost_ = 0, other_ = 0, report_total_ = 0;
  std::vector<double> wakeup_, publish_;
  int64_t t_report_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpMecanumOdomNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
