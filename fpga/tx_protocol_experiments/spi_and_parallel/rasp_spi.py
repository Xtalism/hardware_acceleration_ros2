import struct
import time

import spidev

# Initialize SPI
spi = spidev.SpiDev()
spi.open(1, 2)  # SPI Bus 0, Chip Select 0 (CE0)

# SPI Settings for our FPGA configuration
spi.max_speed_hz = 1000000  # 1MHz
spi.mode = 0  # CPOL=0, CPHA=0


def format_signed_16(high, low):
    """
    Combines two bytes and converts to signed 16-bit integer (Two's Comp
    Uses 'h' format string which natively parses Little Endian signed sh
    """
    # Pack the bytes as little-endian ('<') signed short ('h')
    val = struct.unpack("<h", bytes([low, high]))[0]
    return val


try:
    print("Reading ADXL345 via FPGA SPI...")
    print("---------------------------------")
    while True:
        # We need to read 48 bits (6 bytes): X_L, X_H, Y_L, Y_H, Z_L, Z_H
        # Send dummy bytes (0x00) to clock the data out
        start_time = time.perf_counter()
        resp = spi.xfer2([0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        end_time = time.perf_counter()

        # Calculate latency
        latency_ms = (end_time - start_time) * 1000

        # Parse Data - we swapped them in Verilog so they arrive Low then High
        # resp[0] = X Low, resp[1] = X High
        x = format_signed_16(resp[1], resp[0])

        # resp[2] = Y Low, resp[3] = Y High
        y = format_signed_16(resp[3], resp[2])

        # resp[4] = Z Low, resp[5] = Z High
        z = format_signed_16(resp[5], resp[4])

        print(f"X: {x:6d} | Y: {y:6d} | Z: {z:6d} | {latency_ms:.3f}ms")

        # Poll at ~50Hz
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    spi.close()
