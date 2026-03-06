from time import sleep, time

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

data_pins = [24, 4, 17, 22, 9, 25, 18, 23]
clock_pin = 8
cs_pin = 10

GPIO.setup(clock_pin, GPIO.OUT)
GPIO.setup(cs_pin, GPIO.OUT)
for pin in data_pins:
    GPIO.setup(pin, GPIO.IN)

GPIO.output(clock_pin, GPIO.LOW)
GPIO.output(cs_pin, GPIO.LOW)


def read_all_axes():
    """
    Read all three axes (X, Y, Z) - 6 bytes total.
    """
    # Start transaction
    GPIO.output(cs_pin, GPIO.HIGH)

    # CRITICAL: Wait for FPGA to latch data (at least 2-3 FPGA clock cyc
    # At 50MHz FPGA clock, 1 cycle = 20ns, so wait ~100ns minimum
    sleep(0.0001)  # 100 microseconds to be safe

    bytes_data = []
    for byte_num in range(6):  # Read 6 bytes (3 axes * 2 bytes each)
        # Clock pulse to advance to next byte
        if byte_num > 0:  # Don't clock before first byte
            GPIO.output(clock_pin, GPIO.HIGH)
            sleep(0.000001)  # 1 microsecond pulse width
            GPIO.output(clock_pin, GPIO.LOW)
            sleep(0.000001)  # Wait for data to settle

        # Read the byte
        byte_val = 0
        for i in range(8):
            if GPIO.input(data_pins[i]):
                byte_val |= 1 << i
        bytes_data.append(byte_val)

    # End transaction
    GPIO.output(cs_pin, GPIO.LOW)
    sleep(0.000001)  # Small delay before next transaction

    # Combine bytes into 16-bit signed values
    x_val = bytes_data[0] | (bytes_data[1] << 8)
    y_val = bytes_data[2] | (bytes_data[3] << 8)
    z_val = bytes_data[4] | (bytes_data[5] << 8)

    # Convert to signed integers
    if x_val & 0x8000:
        x_val -= 0x10000
    if y_val & 0x8000:
        y_val -= 0x10000
    if z_val & 0x8000:
        z_val -= 0x10000

    return x_val, y_val, z_val


try:
    print("Reading accelerometer data via parallel interface...")
    print("X, Y, Z should all be 16-bit values\n")

    while True:
        start = time()
        x, y, z = read_all_axes()
        elapsed = (time() - start) * 1000
        print(f"X: {x:6d} | Y: {y:6d} | Z: {z:6d} | {elapsed:.2f}ms")
        sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    GPIO.cleanup()
