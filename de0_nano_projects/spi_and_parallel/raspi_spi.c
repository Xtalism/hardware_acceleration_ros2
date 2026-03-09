#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <signal.h>

// Global flag for clean exit on Ctrl+C
volatile int keep_running = 1;

void int_handler(int dummy) {
    keep_running = 0;
}

// Combines two bytes into a signed 16-bit integer
int16_t format_signed_16(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

int main() {
    signal(SIGINT, int_handler);
    
    // Open SPI device
    // spi.open(1, 2) roughly translates to /dev/spidev1.2 
    int fd = open("/dev/spidev1.2", O_RDWR);
    if (fd < 0) {
        perror("Failed to open SPI bus");
        return 1;
    }

    // Set SPI Mode (CPOL=0, CPHA=0)
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

    // Set max speed (6MHz)
    uint32_t speed = 6000000; 
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("Can't set max speed");
        close(fd);
        return 1;
    }

    printf("Reading ADXL345 via FPGA SPI...\n");
    printf("---------------------------------\n");

    // Buffers for SPI transfer
    uint8_t tx[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[6] = {0};

    // SPI transfer struct
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 6,
        .delay_usecs = 0,
        .speed_hz = speed,
        .bits_per_word = bits,
    };

    struct timespec start, end;

    while (keep_running) {
        // Record start time 
        clock_gettime(CLOCK_MONOTONIC, &start);
        
        // Execute SPI transfer
        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
            perror("SPI transfer failed");
            break;
        }

        // Record end time
        clock_gettime(CLOCK_MONOTONIC, &end);

        // Calculate latency in milliseconds
        double latency_ms = (end.tv_sec - start.tv_sec) * 1000.0 + 
                            (end.tv_nsec - start.tv_nsec) / 1000000.0;

        // Parse Data - arrived Low then High
        int16_t x = format_signed_16(rx[1], rx[0]);
        int16_t y = format_signed_16(rx[3], rx[2]);
        int16_t z = format_signed_16(rx[5], rx[4]);

        printf("X: %6d | Y: %6d | Z: %6d | %.3fms\n", x, y, z, latency_ms);

        // Sleep for 20ms to poll at ~50Hz
        usleep(20000); 
    }

    printf("\nExiting...\n");
    close(fd);
    return 0;
}