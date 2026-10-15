#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>

#define SPI_DEVICE "/dev/spidev1.2"
#define FULL_PACKET_LEN 53  // 6 (IMU) + 47 (LiDAR)
#define LIDAR_PACKET_LEN 47

int lidar_spi_init() {
    int fd = open(SPI_DEVICE, O_RDWR);
    if (fd < 0) return -1;

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 6000000; 

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) return -1;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) return -1;
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) return -1;

    return fd;
}

// Reads the SPI bus and extracts only the 47-byte LiDAR packet
int lidar_read_packet(int fd, uint8_t* lidar_buf) {
    uint8_t rx[FULL_PACKET_LEN] = {0};
    struct spi_ioc_transfer tr = {
        .rx_buf = (unsigned long)rx,
        .len = FULL_PACKET_LEN,
        .speed_hz = 6000000,
        .bits_per_word = 8,
    };

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) return -1;

    // The first 6 bytes are the G-sensor data from the FPGA shift register
    // The LD19 packet starts at index 6
    for (int i = 0; i < LIDAR_PACKET_LEN; i++) {
        lidar_buf[i] = rx[i + 6];
    }

    // Basic header check: LD19 packets always start with 0x54
    if (lidar_buf[0] != 0x54) return -2; 

    return 0;
}

int main() {
    printf("Initializing SPI for LiDAR...\n");
    int spi_fd = lidar_spi_init();
    
    if (spi_fd < 0) {
        perror("Failed to initialize SPI");
        return EXIT_FAILURE;
    }

    uint8_t lidar_buffer[LIDAR_PACKET_LEN];
    printf("SPI Initialized. Listening for packets...\n");

    // Infinite loop to continuously read and print LiDAR packets
    while (1) {
        int result = lidar_read_packet(spi_fd, lidar_buffer);
        
        if (result == 0) {
            // Successfully read a packet starting with 0x54
            printf("Valid LiDAR Packet Received: ");
            // Print the first 10 bytes to verify
            for(int i = 0; i < 10; i++) {
                printf("0x%02X ", lidar_buffer[i]);
            }
            printf("...\n");
        } else if (result == -1) {
            perror("Error reading SPI packet");
            break;
        } else if (result == -2) {
            // Header was not 0x54, print debug info if needed
            printf("Raw SPI data (first 10 bytes): ");
            for(int i = 0; i < 10; i++) {
                printf("0x%02X ", lidar_buffer[i]);
            }
            printf("\n");
        }
        
        // Add a small delay to prevent maxing out the CPU
        usleep(10000); // 10ms
    }

    close(spi_fd);
    return EXIT_SUCCESS;
}