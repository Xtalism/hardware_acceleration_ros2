module rpi_spi_interface (
    input             iRSTN,
    input             iCLK,          // 50MHz FPGA clock
    input      [15:0] iDATA_X,
    input      [15:0] iDATA_Y,
    input      [15:0] iDATA_Z,
    
    // NEW: LiDAR Data inputs from UART Receiver
    input      [7:0]  iLIDAR_BYTE,
    input             iLIDAR_VALID,
    
    // SPI Slave Interface
    input             iSPI_SCK,      // SPI clock from RPi
    input             iSPI_MOSI,     // Master out, slave in (unused)
    output            oSPI_MISO,     // Master in, slave out
    input             iSPI_CS        // Chip select (active low)
);

// Synchronize SPI signals to FPGA clock domain
reg [2:0] sck_sync;
reg [2:0] cs_sync;

always @(posedge iCLK or negedge iRSTN) begin
    if (!iRSTN) begin
        sck_sync <= 3'b000;
        cs_sync <= 3'b111;
    end else begin
        sck_sync <= {sck_sync[1:0], iSPI_SCK};
        cs_sync <= {cs_sync[1:0], iSPI_CS};
    end
end

wire sck_falling = sck_sync[2:1] == 2'b10;
wire cs_active = ~cs_sync[2];

// --- LiDAR Packet Buffer ---
// Holds 47 bytes (376 bits). Bytes shift in as they arrive via UART.
reg [375:0] lidar_buffer;
always @(posedge iCLK or negedge iRSTN) begin
    if (!iRSTN)
        lidar_buffer <= 376'h0;
    else if (iLIDAR_VALID)
        lidar_buffer <= {lidar_buffer[367:0], iLIDAR_BYTE};
end

// --- Expanded Shift Register ---
// 6 bytes (IMU) + 47 bytes (LiDAR) = 53 bytes total (424 bits)
reg [423:0] shift_reg;

always @(posedge iCLK or negedge iRSTN) begin
    if (!iRSTN) begin
        shift_reg <= 424'h0;
    end else if (!cs_active) begin
        // CS inactive: Latch both G-Sensor and LiDAR data
        // Order: X_Low, X_High, Y_Low, Y_High, Z_Low, Z_High, [LiDAR Packet]
        shift_reg <= {iDATA_X[7:0], iDATA_X[15:8],
                      iDATA_Y[7:0], iDATA_Y[15:8],
                      iDATA_Z[7:0], iDATA_Z[15:8],
                      lidar_buffer};
    end else if (sck_falling) begin
        // Shift MSB out to the Pi
        shift_reg <= {shift_reg[422:0], 1'b0};
    end
end

// Zero latency assignment for MISO
assign oSPI_MISO = cs_active ? shift_reg[423] : 1'bz;

endmodule