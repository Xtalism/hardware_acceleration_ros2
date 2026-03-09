#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "common.h"
#include "adxl345_fpga.h"

static int16_t format_signed_16(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

int spi_init(const char* device) {
    int fd = open(device, O_RDWR);
    if (fd < 0) return ERROR;

    uint8_t mode = SPI_MODE_0;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) { close(fd); return ERROR; }

    uint8_t bits = 8;
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) { close(fd); return ERROR; }

    uint32_t speed = 6000000; 
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) { close(fd); return ERROR; }

    return fd;
}

int spi_read_data(int fd, float* g_x, float* g_y, float* g_z) {
    uint8_t tx[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[6] = {0};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 6,
        .delay_usecs = 0,
        .speed_hz = 6000000,
        .bits_per_word = 8,
    };

    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) return ERROR;

    int16_t raw_x = format_signed_16(rx[1], rx[0]);
    int16_t raw_y = format_signed_16(rx[3], rx[2]);
    int16_t raw_z = format_signed_16(rx[5], rx[4]);
    
    int16_t offset_x = 7;
    int16_t offset_y = -11;
    int16_t offset_z = -6; 
    
    float scale_factor = 1.0 / 256.0;

    *g_x = (raw_x - offset_x) * scale_factor;
    *g_y = (raw_y - offset_y) * scale_factor;
    *g_z = (raw_z - offset_z) * scale_factor;

    return SUCCESS;
}

void spi_close(int fd) {
    if (fd >= 0) close(fd);
}