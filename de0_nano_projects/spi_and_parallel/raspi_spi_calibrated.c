#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <signal.h>

volatile int keep_running = 1;

void int_handler(int dummy) {
    keep_running = 0;
}

int16_t format_signed_16(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

int main() {
    signal(SIGINT, int_handler);
    
    int fd = open("/dev/spidev1.2", O_RDWR);
    if (fd < 0) {
        perror("Failed to open SPI bus");
        return 1;
    }

    uint8_t mode = SPI_MODE_0;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("Can't set SPI mode");
        close(fd);
        return 1;
    }

    // Set bits per word (8 bits is standard)
    uint8_t bits = 8;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
        perror("Can't set bits per word");
        close(fd);
        return 1;
    }

    uint32_t speed = 6000000; 
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("Can't set max speed");
        close(fd);
        return 1;
    }

    printf("Reading ADXL345 via FPGA SPI...\n");
    printf("---------------------------------\n");

    uint8_t tx[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[6] = {0};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 6,
        .delay_usecs = 0,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    struct timespec start, end;

    int16_t offset_x = 7;
    int16_t offset_y = -11;
    int16_t offset_z = -6;
    
    // Convert LSB to Gs (1g = 256 LSB in standard ADXL345 configuration)
    float scale_factor = 1.0 / 256.0;

    while (keep_running) {
        clock_gettime(CLOCK_MONOTONIC, &start);
        
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
            perror("SPI transfer failed");
            break;
        }

        clock_gettime(CLOCK_MONOTONIC, &end);

        double latency_ms = (end.tv_sec - start.tv_sec) * 1000.0 + 
                            (end.tv_nsec - start.tv_nsec) / 1000000.0;

        int16_t raw_x = format_signed_16(rx[1], rx[0]);
        int16_t raw_y = format_signed_16(rx[3], rx[2]);
        int16_t raw_z = format_signed_16(rx[5], rx[4]);
        
        int16_t cal_x = raw_x - offset_x;
        int16_t cal_y = raw_y - offset_y;
        int16_t cal_z = raw_z - offset_z;

        float g_x = cal_x * scale_factor;
        float g_y = cal_y * scale_factor;
        float g_z = cal_z * scale_factor;

        printf("X: %5.2fg | Y: %5.2fg | Z: %5.2fg | %.3fms\n", g_x, g_y, g_z, latency_ms);

        usleep(20000); 
    }

    printf("\nExiting...\n");
    close(fd);
    return 0;
}