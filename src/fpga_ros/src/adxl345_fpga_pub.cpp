#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "adxl345_fpga.h"
#include "common.h"

using namespace std::chrono_literals;

class Adxl345Node : public rclcpp::Node {
public:
    Adxl345Node() : Node("adxl345_fpga_node") {
        spi_fd_ = spi_init("/dev/spidev1.2");
        if (spi_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init SPI");
            throw std::runtime_error("SPI Init Failed");
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&Adxl345Node::timer_callback, this));
    }

    ~Adxl345Node() {
        spi_close(spi_fd_);
    }

private:
    void timer_callback() {
        float x, y, z;
        if (spi_read_data(spi_fd_, &x, &y, &z) == SUCCESS) {
            auto msg = sensor_msgs::msg::Imu();
            msg.header.stamp = this->now();
            msg.header.frame_id = "imu_link";
            
            msg.linear_acceleration.x = x * 9.80665;
            msg.linear_acceleration.y = y * 9.80665;
            msg.linear_acceleration.z = z * 9.80665;
            
            msg.angular_velocity_covariance[0] = -1.0;
            msg.orientation_covariance[0] = -1.0;

            publisher_->publish(msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to read SPI data");
        }
    }

    int spi_fd_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Adxl345Node>());
    rclcpp::shutdown();
    return 0;
}
