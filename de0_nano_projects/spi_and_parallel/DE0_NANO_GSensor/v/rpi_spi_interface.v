module rpi_spi_interface (
    input             iRSTN,
    input             iCLK,          // 50MHz FPGA clock
    input      [15:0] iDATA_X,
    input      [15:0] iDATA_Y,
    input      [15:0] iDATA_Z,
    
    // SPI Slave Interface
    input             iSPI_SCK,      // SPI clock from RPi
    input             iSPI_MOSI,     // Master out, slave in (unused for read-only)
    output            oSPI_MISO,     // Master in, slave out
    input             iSPI_CS        // Chip select (active low)
);

// Synchronize SPI signals to FPGA clock domain
reg [2:0] sck_sync;
reg [2:0] cs_sync;
reg [2:0] mosi_sync;

always @(posedge iCLK or negedge iRSTN) begin
    if (!iRSTN) begin
        sck_sync <= 3'b000;
        cs_sync <= 3'b111;
        mosi_sync <= 3'b000;
    end else begin
        sck_sync <= {sck_sync[1:0], iSPI_SCK};
        cs_sync <= {cs_sync[1:0], iSPI_CS};
        mosi_sync <= {mosi_sync[1:0], iSPI_MOSI};
    end
end

wire sck_falling = sck_sync[2:1] == 2'b10;
wire cs_active = ~cs_sync[2];

// Data register
reg [47:0] shift_reg;

// In SPI Mode 0, master samples on the rising edge, and slave shifts on the falling edge.
always @(posedge iCLK or negedge iRSTN) begin
    if (!iRSTN) begin
        shift_reg <= 48'h0;
    end else if (!cs_active) begin
        // CS inactive - Load the data in X, Y, Z order. 
        // CRITICAL FIX: Load Low byte [7:0] first, then High Byte [15:8] (Little Endian!)
        shift_reg <= {iDATA_X[7:0], iDATA_X[15:8],
                      iDATA_Y[7:0], iDATA_Y[15:8],
                      iDATA_Z[7:0], iDATA_Z[15:8]};
    end else if (sck_falling) begin
        // Shift data bits to the left EXACTLY on the falling edge. 
        shift_reg <= {shift_reg[46:0], 1'b0};
    end
end

// Direct wire assignment ensures zero clock latency when CS drops.
assign oSPI_MISO = cs_active ? shift_reg[47] : 1'bz;

endmodule