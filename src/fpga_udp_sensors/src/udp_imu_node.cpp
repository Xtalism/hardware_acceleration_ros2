// Receives the ADXL345 samples the DE0-Nano sends as UDP broadcasts
// (ethernet_tx_paket/fpga/gsensor_udp) and publishes them as
// sensor_msgs/Imu on imu/data, stamped with the kernel's receive time of each
// packet. Orientation is the tilt (roll, pitch) from gravity; yaw is unknown.
//
// Every stats_period seconds it logs how regularly the device sampled and how
// regularly the packets reached this PC. Any device that sends the same packet
// (e.g. an MCU) is measured the same way, so the numbers can be compared.
//
// UDP payload, 18 bytes, big-endian:
//   "GS", u32 sequence, u32 device time (us), i16 X, i16 Y, i16 Z, u8 DEVID, u8 flags

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace
{
constexpr double kGPerLsb = 1.0 / 256.0;   // ADXL345 full resolution, 3.9 mg/LSB
constexpr double kGravity = 9.80665;
constexpr size_t kPayloadSize = 18;

int64_t to_ns(const timespec & ts) {return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;}

int64_t now_ns()
{
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);   // same clock as the kernel receive timestamp
  return to_ns(ts);
}

uint32_t be32(const uint8_t * p) {return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | p[3];}
int16_t be16(const uint8_t * p) {return int16_t(uint16_t(p[0]) << 8 | p[1]);}

double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {return 0.0;}
  size_t i = std::min(v.size() - 1, size_t(p * double(v.size() - 1) + 0.5));
  std::nth_element(v.begin(), v.begin() + i, v.end());
  return v[i];
}

struct Sample
{
  double dev_us;      // device time, unwrapped
  int64_t t_kernel;   // packet reached the kernel (SO_TIMESTAMPNS)
  int64_t t_node;     // recvmsg() returned in this node
  int64_t t_pub;      // publish() returned
};
}  // namespace

class UdpImuNode : public rclcpp::Node
{
public:
  UdpImuNode()
  : Node("udp_imu")
  {
    port_ = declare_parameter("port", 1024);
    frame_id_ = declare_parameter("frame_id", std::string("imu_link"));
    parent_frame_ = declare_parameter("parent_frame", std::string("world"));
    auto position = declare_parameter("position", std::vector<double>{0.0, 0.0, 0.0});
    if (position.size() != 3) {
      throw std::invalid_argument("position must be [x, y, z] in metres");
    }
    stats_period_ = declare_parameter("stats_period", 2.0);

    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::QoS(10));

    // imu_link sits at `position` in parent_frame; the tilt is in the message
    tf_static_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = parent_frame_;
    t.child_frame_id = frame_id_;
    t.transform.translation.x = position[0];
    t.transform.translation.y = position[1];
    t.transform.translation.z = position[2];
    t.transform.rotation.w = 1.0;
    tf_static_->sendTransform(t);

    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    int on = 1, rcvbuf = 1 << 20;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
    setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &on, sizeof(on));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(uint16_t(port_));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);   // also receives broadcasts
    if (fd_ < 0 || bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error("cannot bind UDP port " + std::to_string(port_) + ": " + strerror(errno));
    }

    RCLCPP_INFO(
      get_logger(), "listening on UDP port %ld, publishing %s (frame %s)",
      port_, pub_->get_topic_name(), frame_id_.c_str());
    t_report_ = now_ns();
    thread_ = std::thread(&UdpImuNode::rx_loop, this);
  }

  ~UdpImuNode() override
  {
    running_ = false;
    if (thread_.joinable()) {thread_.join();}
    if (fd_ >= 0) {close(fd_);}
    RCLCPP_INFO(
      get_logger(), "total: %lu samples, %lu lost, %lu dropped in the device, %lu other packets, "
      "worst arrival delay %.0f us, worst node wake-up %.0f us",
      total_, lost_, dropped_, bad_, worst_spread_us_, worst_wakeup_us_);
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
    if (n != kPayloadSize || b[0] != 'G' || b[1] != 'S') {
      ++bad_;
      return;
    }
    uint32_t seq = be32(b + 2);
    uint32_t dev_us = be32(b + 6);
    double ax = be16(b + 10) * kGPerLsb, ay = be16(b + 12) * kGPerLsb, az = be16(b + 14) * kGPerLsb;
    devid_ = b[16];
    dropped_ += b[17] & 1;

    if (have_prev_ && seq - prev_seq_ != 0 && seq - prev_seq_ < 1000000) {
      lost_ += seq - prev_seq_ - 1;
      dev_time_us_ += double(uint32_t(dev_us - prev_dev_us_));
    } else if (have_prev_) {
      RCLCPP_WARN(get_logger(), "sequence jumped %u -> %u: device restarted", prev_seq_, seq);
      win_.clear();
      dev_time_us_ = 0.0;
    }
    have_prev_ = true;
    prev_seq_ = seq;
    prev_dev_us_ = dev_us;

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(t_kernel, RCL_SYSTEM_TIME);
    msg.header.frame_id = frame_id_;
    msg.linear_acceleration.x = ax * kGravity;
    msg.linear_acceleration.y = ay * kGravity;
    msg.linear_acceleration.z = az * kGravity;
    for (int i : {0, 4, 8}) {msg.linear_acceleration_covariance[i] = 0.006;}   // ~8 mg noise

    // tilt from gravity (only valid while the board isn't accelerating)
    double roll = std::atan2(ay, az);
    double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
    double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
    msg.orientation.w = cr * cp;
    msg.orientation.x = sr * cp;
    msg.orientation.y = cr * sp;
    msg.orientation.z = -sr * sp;
    msg.orientation_covariance[0] = msg.orientation_covariance[4] = 0.001;
    msg.orientation_covariance[8] = 1e6;          // yaw unknown
    msg.angular_velocity_covariance[0] = -1.0;    // no gyro

    pub_->publish(msg);
    win_.push_back({dev_time_us_, t_kernel, t_node, now_ns()});
    ++total_;
  }

  void report()
  {
    double period_s = (now_ns() - t_report_) / 1e9;
    t_report_ = now_ns();
    size_t n = win_.size();
    if (n < 3) {
      RCLCPP_WARN(
        get_logger(), "%zu samples in %.1f s on UDP port %ld%s", n, period_s, port_,
        bad_ ? " (other packets seen: is the G-sensor design loaded?)" : "");
      win_.clear();
      return;
    }

    std::vector<double> dev_dt, rx_dt, spread, wakeup, publish;
    for (size_t i = 1; i < n; ++i) {
      dev_dt.push_back(win_[i].dev_us - win_[i - 1].dev_us);
      rx_dt.push_back((win_[i].t_kernel - win_[i - 1].t_kernel) / 1e3);
    }

    // Arrival delay = kernel time - device time. Its absolute value is unknown
    // (different clocks) and the clocks drift, so fit a line and report how much
    // later than the earliest packet each one arrived.
    std::vector<double> x(n), d(n);
    for (size_t i = 0; i < n; ++i) {
      x[i] = win_[i].dev_us - win_[0].dev_us;
      d[i] = (win_[i].t_kernel - win_[0].t_kernel) / 1e3 - x[i];
      wakeup.push_back((win_[i].t_node - win_[i].t_kernel) / 1e3);
      publish.push_back((win_[i].t_pub - win_[i].t_node) / 1e3);
    }
    double mx = 0, md = 0, sxx = 0, sxd = 0;
    for (size_t i = 0; i < n; ++i) {mx += x[i] / n; md += d[i] / n;}
    for (size_t i = 0; i < n; ++i) {sxx += (x[i] - mx) * (x[i] - mx); sxd += (x[i] - mx) * (d[i] - md);}
    double slope = sxx > 0 ? sxd / sxx : 0.0;
    double rmin = 1e300;
    for (size_t i = 0; i < n; ++i) {
      spread.push_back(d[i] - (md + slope * (x[i] - mx)));
      rmin = std::min(rmin, spread.back());
    }
    for (double & s : spread) {s -= rmin;}

    double mean = 0, var = 0;
    for (double v : dev_dt) {mean += v / dev_dt.size();}
    for (double v : dev_dt) {var += (v - mean) * (v - mean) / dev_dt.size();}
    double rx_mean = 0, rx_var = 0;
    for (double v : rx_dt) {rx_mean += v / rx_dt.size();}
    for (double v : rx_dt) {rx_var += (v - rx_mean) * (v - rx_mean) / rx_dt.size();}

    double spread_max = percentile(spread, 1.0), wake_max = percentile(wakeup, 1.0);
    worst_spread_us_ = std::max(worst_spread_us_, spread_max);
    worst_wakeup_us_ = std::max(worst_wakeup_us_, wake_max);

    RCLCPP_INFO(
      get_logger(),
      "%.1f Hz  lost %lu  DEVID %02X | device sampling: every %.1f us, jitter sd %.1f (%.0f..%.0f) | "
      "PC arrival: jitter sd %.1f us, delay vs best p50 %.0f p99 %.0f max %.0f us | "
      "node wake-up p50 %.0f p99 %.0f max %.0f us, publish %.0f us | clock drift %+.0f ppm",
      1e6 / rx_mean, lost_, devid_,
      mean, std::sqrt(var), percentile(dev_dt, 0.0), percentile(dev_dt, 1.0),
      std::sqrt(rx_var), percentile(spread, 0.5), percentile(spread, 0.99), spread_max,
      percentile(wakeup, 0.5), percentile(wakeup, 0.99), wake_max, percentile(publish, 0.5),
      slope * 1e6);
    win_.clear();
  }

  int64_t port_;
  std::string frame_id_, parent_frame_;
  double stats_period_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

  int fd_ = -1;
  std::thread thread_;
  std::atomic<bool> running_{true};

  bool have_prev_ = false;
  uint32_t prev_seq_ = 0, prev_dev_us_ = 0;
  uint8_t devid_ = 0;
  double dev_time_us_ = 0.0;
  uint64_t total_ = 0, lost_ = 0, dropped_ = 0, bad_ = 0;
  double worst_spread_us_ = 0.0, worst_wakeup_us_ = 0.0;
  std::vector<Sample> win_;
  int64_t t_report_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpImuNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
