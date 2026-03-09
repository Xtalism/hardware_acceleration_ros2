#ifndef ADXL345_FPGA_H
#define ADXL345_FPGA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int spi_init(const char* device);
int spi_read_data(int fd, float* g_x, float* g_y, float* g_z);
void spi_close(int fd);

#ifdef __cplusplus
}
#endif

#endif