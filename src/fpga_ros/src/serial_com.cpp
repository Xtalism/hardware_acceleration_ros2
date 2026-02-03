#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <getopt.h>
#include <map>
#include <poll.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int readFromSerialPort(int fd, char* buffer, size_t size);

class ROS2_Publisher : public rclcpp::Node {
public:
    ROS2_Publisher(int serial_fd) : Node("fpga_node"), serial_fd_(serial_fd) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/fpga/read", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ROS2_Publisher::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        char buffer[4096];
        int n = readFromSerialPort(serial_fd_, buffer, sizeof(buffer));
        RCLCPP_INFO(this->get_logger(), "Timer callback: read %d bytes", n);
        if (n > 0) {
            std_msgs::msg::String msg;
            msg.data = std::string(buffer, n);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
            publisher_->publish(msg);
        }
    }
    int serial_fd_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

speed_t getBaudRate(const char* baudStr) {
    static const std::map<int, speed_t> baudMap = {
        {9600, B9600},
        {19200, B19200},
        {38400, B38400},
        {57600, B57600},
        {115200, B115200},
        {230400, B230400},
        {460800, B460800},
        {500000, B500000},
        {576000, B576000},
        {921600, B921600},
        {1000000, B1000000},
        {1152000, B1152000},
    };
    
    int baudInt = atoi(baudStr);
    auto it = baudMap.find(baudInt);
    if (it != baudMap.end()) {
        return it->second;
    }
    return B0;
}

int openSerialPort(const char* portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portname << ": "
             << strerror(errno) << std::endl;
        return -1;
    }
    return fd;
}

bool configureSerialPort(int fd, speed_t speed)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno)
             << std::endl;
        return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    
    // Critical for speed: non-blocking reads
    tty.c_cc[VMIN] = 1;  // Return immediately with at least 1 byte
    tty.c_cc[VTIME] = 0; // No timeout - pure non-blocking

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

int readFromSerialPort(int fd, char* buffer, size_t size)
{
    return read(fd, buffer, size);
}

int writeToSerialPort(int fd, const char* buffer, size_t size) {
    return write(fd, buffer, size);
}

void closeSerialPort(int fd) { close(fd); }

void print_usage(char *argv[]) {
    printf("Usage: %s -p <serial port> -b <baud rate>\n", argv[0]);
    printf("\t -p - (required) device serial port\n");
    printf("\t -b - baud rate (default: 9600)\n");
    return;
}

int main (int argc, char *argv[]) {
    char *port = NULL;
    char *baud = NULL;
    int fd = -1;

    int c;

    while ((c = getopt(argc, argv, "p:b:")) != -1) {
      switch (c) {
          case 'p':
              port = optarg;
                 break;
          case 'b':
              baud = optarg;
                 break;
          default:
                 break;
      }
    }

    if (port == NULL) {
        printf("Serial port is a required argument\n");
        print_usage(argv);
        return 1;
    }

    fd = openSerialPort(port);
    if (fd < 0) 
        return 1;

    if (baud) {
        speed_t baudRate = getBaudRate(baud);
        if (!configureSerialPort(fd, baudRate)) {
            closeSerialPort(fd);
            return 1;
        }
    } else {
        if (!configureSerialPort(fd, B9600)) {
            closeSerialPort(fd);
            return 1;
        }
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROS2_Publisher>(fd);
    rclcpp::spin(node);
    rclcpp::shutdown();

    closeSerialPort(fd);
    return 0;
}
