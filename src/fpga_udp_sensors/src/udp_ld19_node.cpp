// Receives the LD19 lidar packets the DE0-Nano sends as UDP broadcasts
// (ethernet_tx_paket/fpga/ld19_udp or fpga/sensors_udp)
// and publishes one sensor_msgs/LaserScan per rotation on "scan".
//
// UDP payload, 59 bytes: "LD", u32 sequence, u32 FPGA time (us, when the LD19
// packet's first byte reached the FPGA), u16 bad-CRC count (big-endian), then
// the 47-byte LD19 packet (LD19 Development Manual 3.1, little-endian):
//   0x54, 0x2C, speed (deg/s), start angle (0.01 deg),
//   12 x (distance mm, intensity), end angle, timestamp (ms), CRC-8
//
// The LD19 measures angles clockwise from its front; ROS angles grow
// counter-clockwise (REP 103), so a point at LD19 angle a is published at -a.
// Distance 0 means no return and is published as NaN. Every scan has the same
// number of beams (slam_toolbox needs that): the beams parameter, or 0 to use
// the point count of the first complete rotation.
//
// Every stats_period seconds it logs the scan rate and how regularly the
// packets reached this PC, measured like udp_imu does for the G-sensor.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace
{
constexpr size_t kPayloadSize = 59;
constexpr size_t kHeaderSize = 12;
constexpr int kPoints = 12;
// 47 bytes at 230400 baud, 8N1: the packet arrives this long after its first byte left the LD19
constexpr int64_t kPacketUartNs = 47LL * 10 * 1000000000LL / 230400;

int64_t to_ns(const timespec & ts) {return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;}

int64_t now_ns()
{
  timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);   // same clock as the kernel receive timestamp
  return to_ns(ts);
}

uint32_t be32(const uint8_t * p) {return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | p[3];}
uint16_t be16(const uint8_t * p) {return uint16_t(p[0] << 8 | p[1]);}
uint16_t le16(const uint8_t * p) {return uint16_t(p[1] << 8 | p[0]);}

// LD19 CRC-8: polynomial 0x4D, initial 0 (same as the manual's CrcTable)
uint8_t crc8(const uint8_t * p, size_t n)
{
  uint8_t c = 0;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k) {c = (c & 0x80) ? uint8_t((c << 1) ^ 0x4D) : uint8_t(c << 1);}
  }
  return c;
}

double percentile(std::vector<double> v, double p)
{
  if (v.empty()) {return 0.0;}
  size_t i = std::min(v.size() - 1, size_t(p * double(v.size() - 1) + 0.5));
  std::nth_element(v.begin(), v.begin() + i, v.end());
  return v[i];
}

struct Point
{
  float angle;       // ROS angle, rad, [0, 2 pi)
  float range;       // m, NaN = no return
  float intensity;
};

struct Sample
{
  double dev_us;      // FPGA time, unwrapped
  int64_t t_kernel;   // packet reached the kernel (SO_TIMESTAMPNS)
  int64_t t_node;     // recvmsg() returned in this node
};
}  // namespace

class UdpLd19Node : public rclcpp::Node
{
public:
  UdpLd19Node()
  : Node("udp_ld19")
  {
    port_ = declare_parameter("port", 1025);
    frame_id_ = declare_parameter("frame_id", std::string("laser_link"));
    parent_frame_ = declare_parameter("parent_frame", std::string("world"));
    auto position = declare_parameter("position", std::vector<double>{0.0, 0.0, 0.0});
    if (position.size() != 3) {
      throw std::invalid_argument("position must be [x, y, z] in metres");
    }
    range_min_ = declare_parameter("range_min", 0.02);
    range_max_ = declare_parameter("range_max", 12.0);
    beams_ = size_t(std::max<int64_t>(0, declare_parameter("beams", 0)));
    stats_period_ = declare_parameter("stats_period", 2.0);

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(10));

    // laser_link sits at `position` in parent_frame
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
      get_logger(), "listening on UDP port %ld, publishing %s (frame %s, range %.2f..%.1f m)",
      port_, pub_->get_topic_name(), frame_id_.c_str(), range_min_, range_max_);
    t_report_ = now_ns();
    thread_ = std::thread(&UdpLd19Node::rx_loop, this);
  }

  ~UdpLd19Node() override
  {
    running_ = false;
    if (thread_.joinable()) {thread_.join();}
    if (fd_ >= 0) {close(fd_);}
    RCLCPP_INFO(
      get_logger(), "total: %lu packets, %lu scans, %lu lost, %u bad CRC in the FPGA, %lu bad CRC on the PC, "
      "%lu other packets", total_, scans_, lost_, fpga_bad_, pc_bad_, other_);
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
    if (n != kPayloadSize || b[0] != 'L' || b[1] != 'D') {
      ++other_;
      return;
    }
    uint32_t seq = be32(b + 2);
    uint32_t dev_us = be32(b + 6);
    fpga_bad_ = be16(b + 10);
    const uint8_t * p = b + kHeaderSize;
    if (p[0] != 0x54 || p[1] != 0x2C || crc8(p, 46) != p[46]) {   // the FPGA checked it already
      ++pc_bad_;
      return;
    }
    uint16_t speed = le16(p + 2), start = le16(p + 4), end = le16(p + 42);

    if (have_prev_ && seq - prev_seq_ != 0 && seq - prev_seq_ < 1000000) {
      lost_ += seq - prev_seq_ - 1;
      dev_time_us_ += double(uint32_t(dev_us - prev_dev_us_));
    } else if (have_prev_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "sequence jumped %u -> %u: FPGA restarted", prev_seq_, seq);
      win_.clear();
      dev_time_us_ = 0.0;
    }
    have_prev_ = true;
    prev_seq_ = seq;
    prev_dev_us_ = dev_us;
    ++total_;
    win_.push_back({dev_time_us_, t_kernel, t_node});

    // a rotation ends when the start angle wraps past 0
    if (!scan_.empty() && start < prev_start_) {publish_scan();}
    prev_start_ = start;
    if (speed > 0) {speed_ = speed;}

    double span = end >= start ? end - start : end + 36000.0 - start;   // 0.01 deg
    if (scan_.empty()) {
      // the packet's first byte left the LD19 right after its last point was measured
      scan_stamp_ns_ = t_kernel - kPacketUartNs - int64_t(span / 100.0 / speed_ * 1e9);
    }
    for (int i = 0; i < kPoints; ++i) {
      uint16_t dist = le16(p + 6 + 3 * i);
      double a = std::fmod((start + span * i / (kPoints - 1)) / 100.0, 360.0);   // LD19, clockwise
      double ros = std::fmod(360.0 - a, 360.0) * M_PI / 180.0;
      scan_.push_back({float(ros), dist ? dist / 1000.0f : std::numeric_limits<float>::quiet_NaN(),
          float(p[8 + 3 * i])});
    }
  }

  void publish_scan()
  {
    if (synced_ && beams_ == 0 && scan_.size() >= 2 * kPoints) {
      beams_ = scan_.size();                     // first complete rotation sets the beam count
      RCLCPP_INFO(get_logger(), "%zu beams per scan (%.3f deg)", beams_, 360.0 / beams_);
    }
    size_t n = beams_;
    if (synced_ && n > 0 && scan_.size() >= 2 * kPoints) {   // the first rotation is incomplete: skip it
      int64_t t0 = now_ns();
      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = rclcpp::Time(scan_stamp_ns_, RCL_SYSTEM_TIME);
      msg.header.frame_id = frame_id_;
      msg.angle_min = 0.0f;
      msg.angle_increment = float(2.0 * M_PI / n);
      msg.angle_max = msg.angle_increment * float(n - 1);
      msg.scan_time = float(360.0 / speed_);
      msg.time_increment = msg.scan_time / float(n);
      msg.range_min = float(range_min_);
      msg.range_max = float(range_max_);
      msg.ranges.assign(n, std::numeric_limits<float>::quiet_NaN());
      msg.intensities.assign(n, 0.0f);
      for (const Point & pt : scan_) {
        size_t i = size_t(std::lround(pt.angle / msg.angle_increment)) % n;
        if (std::isnan(msg.ranges[i]) || pt.range < msg.ranges[i]) {   // keep the nearer of two
          msg.ranges[i] = pt.range;
          msg.intensities[i] = pt.intensity;
        }
      }
      pub_->publish(msg);
      ++scans_;
      win_points_ += scan_.size();
      win_scans_ += 1;
      publish_us_.push_back((now_ns() - t0) / 1e3);
    }
    synced_ = true;
    scan_.clear();
  }

  void report()
  {
    double period_s = (now_ns() - t_report_) / 1e9;
    t_report_ = now_ns();
    size_t n = win_.size();
    if (n < 3) {
      RCLCPP_WARN(
        get_logger(), "%zu packets in %.1f s on UDP port %ld%s", n, period_s, port_,
        other_ ? " (other packets seen: is a design with the LD19 loaded?)" : "");
      win_.clear();
      return;
    }

    // Arrival delay = kernel time - FPGA time. Its absolute value is unknown
    // (different clocks) and the clocks drift, so fit a line and report how much
    // later than the earliest packet each one arrived.
    std::vector<double> x(n), d(n), spread, wakeup, rx_dt;
    for (size_t i = 0; i < n; ++i) {
      x[i] = win_[i].dev_us - win_[0].dev_us;
      d[i] = (win_[i].t_kernel - win_[0].t_kernel) / 1e3 - x[i];
      wakeup.push_back((win_[i].t_node - win_[i].t_kernel) / 1e3);
      if (i) {rx_dt.push_back((win_[i].t_kernel - win_[i - 1].t_kernel) / 1e3);}
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
    double rx_mean = 0;
    for (double v : rx_dt) {rx_mean += v / rx_dt.size();}

    RCLCPP_INFO(
      get_logger(),
      "%.2f scans/s, %.0f pkt/s, %.0f points/s, %.0f points/scan, lost %lu, bad CRC %u | "
      "arrival vs FPGA time: later than the best packet p50 %.0f p99 %.0f max %.0f us | "
      "node wake-up p50 %.0f p99 %.0f max %.0f us, scan publish %.0f us | clock drift %+.0f ppm",
      win_scans_ / period_s, 1e6 / rx_mean, 12e6 / rx_mean,
      win_scans_ ? double(win_points_) / win_scans_ : 0.0, lost_, fpga_bad_,
      percentile(spread, 0.5), percentile(spread, 0.99), percentile(spread, 1.0),
      percentile(wakeup, 0.5), percentile(wakeup, 0.99), percentile(wakeup, 1.0),
      percentile(publish_us_, 0.5), slope * 1e6);
    win_.clear();
    publish_us_.clear();
    win_points_ = win_scans_ = 0;
  }

  int64_t port_;
  std::string frame_id_, parent_frame_;
  double range_min_, range_max_, stats_period_;
  size_t beams_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_;

  int fd_ = -1;
  std::thread thread_;
  std::atomic<bool> running_{true};

  bool have_prev_ = false;
  uint32_t prev_seq_ = 0, prev_dev_us_ = 0;
  double dev_time_us_ = 0.0;
  uint16_t prev_start_ = 0;
  bool synced_ = false;
  double speed_ = 3600.0;                // deg/s, updated from the packets
  std::vector<Point> scan_;
  int64_t scan_stamp_ns_ = 0;

  uint64_t total_ = 0, lost_ = 0, pc_bad_ = 0, other_ = 0, scans_ = 0;
  uint16_t fpga_bad_ = 0;
  std::vector<Sample> win_;
  std::vector<double> publish_us_;
  size_t win_points_ = 0, win_scans_ = 0;
  int64_t t_report_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UdpLd19Node>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
