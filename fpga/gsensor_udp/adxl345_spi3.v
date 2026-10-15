//=============================================================================
// ADXL345 reader for the DE0-Nano, 3-wire SPI (SDIO is I2C_SDAT, shared
// with the EEPROM; SCLK is I2C_SCLK).
//
// Configures the sensor, reads DEVID, then polls INT_SOURCE and does one
// 6-byte burst read of DATAX0..DATAZ1 every time DATA_READY is set, so every
// sample is read exactly once at the output data rate. The INT pin is not
// used. If no sample arrives for ~100 ms the sensor is configured again.
//
// SPI mode 3 (SCLK idles high, data changes on the falling edge, sampled on
// the rising edge), MSB first, SCLK = i_clk / (2 * HALF_CYCLES).
//=============================================================================

module adxl345_spi3
#(
    parameter        HALF_CYCLES    = 12,        // 50 MHz / 24 = 2.08 MHz (ADXL345 max 5 MHz)
    parameter        STARTUP_CYCLES = 500_000,   // 10 ms before the first access
    parameter        TIMEOUT_CYCLES = 5_000_000, // 100 ms without DATA_READY -> configure again
    parameter [7:0]  BW_RATE        = 8'h0C,     // 0x0A 100 Hz, 0x0B 200 Hz, 0x0C 400 Hz, 0x0D 800 Hz
    parameter [7:0]  DATA_FORMAT    = 8'h4B      // 3-wire SPI, full resolution (3.9 mg/LSB), +-16 g
)
(
    input  wire        i_clk,
    input  wire        i_reset,

    output reg         o_spi_cs_n,
    output reg         o_spi_sclk,
    inout  wire        io_spi_sdio,

    output reg         o_valid,         // one-clock pulse: new o_x/o_y/o_z
    output reg  [15:0] o_x,
    output reg  [15:0] o_y,
    output reg  [15:0] o_z,
    output reg  [7:0]  o_devid          // 0xE5 when the sensor answers
);

localparam R_DEVID       = 6'h00;
localparam R_BW_RATE     = 6'h2C;
localparam R_POWER_CTL   = 6'h2D;
localparam R_INT_ENABLE  = 6'h2E;
localparam R_INT_SOURCE  = 6'h30;
localparam R_DATA_FORMAT = 6'h31;
localparam R_DATAX0      = 6'h32;
localparam R_FIFO_CTL    = 6'h38;

localparam E_IDLE = 3'd0, E_SETUP = 3'd1, E_LOW = 3'd2, E_HIGH = 3'd3, E_HOLD = 3'd4, E_GAP = 3'd5;
localparam S_START = 3'd0, S_CFG = 3'd1, S_ID = 3'd2, S_POLL = 3'd3, S_READ = 3'd4;
localparam N_CFG = 3'd6;

// sequencer -> engine request
reg  [55:0] req_bits;               // command byte, then write byte (MSB first)
reg  [5:0]  req_nbits;
reg         req_read;
reg         e_start = 0;

// engine
reg  [2:0]  e_st = E_IDLE;
reg  [7:0]  div  = 0;
reg  [5:0]  bit_n;
reg  [55:0] tx_sh;
reg  [47:0] rx_sh;
reg         e_done   = 0;
reg         sdio_oe  = 0;
reg         sdio_out = 1;

// sequencer
reg  [2:0]  s_st   = S_START;
reg  [2:0]  s_idx  = 0;
reg         s_busy = 0;
reg  [22:0] s_wait = 0;             // cycles since reset / last sample (saturates)
reg  [13:0] cfg;

wire half_done = (div == HALF_CYCLES - 1);

assign io_spi_sdio = sdio_oe ? sdio_out : 1'bz;

//=============================================================================
// SPI transaction engine: 8-bit command, then 1 write byte or N read bytes
//=============================================================================
always @(posedge i_clk)
begin
    e_done <= 1'b0;
    if (i_reset) begin
        e_st       <= E_IDLE;
        div        <= 0;
        o_spi_cs_n <= 1'b1;
        o_spi_sclk <= 1'b1;
        sdio_oe    <= 1'b0;
    end else begin
        if (e_st != E_IDLE) div <= half_done ? 8'd0 : div + 1'b1;

        case (e_st)
        E_IDLE:
            if (e_start) begin
                tx_sh      <= req_bits;
                o_spi_cs_n <= 1'b0;            // CS falls while SCLK is high
                bit_n      <= 0;
                div        <= 0;
                e_st       <= E_SETUP;
            end

        E_SETUP:
            if (half_done) begin
                o_spi_sclk <= 1'b0;            // first falling edge, drive command bit 7
                sdio_oe    <= 1'b1;
                sdio_out   <= tx_sh[55];
                e_st       <= E_LOW;
            end

        E_LOW:
            if (half_done) begin
                o_spi_sclk <= 1'b1;            // rising edge: both sides sample
                if (req_read && bit_n >= 8) rx_sh <= {rx_sh[46:0], io_spi_sdio};
                e_st       <= E_HIGH;
            end

        E_HIGH:
        begin
            // hand SDIO to the sensor just before the first read bit's falling edge
            if (req_read && bit_n == 7 && div == HALF_CYCLES - 2) sdio_oe <= 1'b0;

            if (half_done) begin
                if (bit_n == req_nbits - 1) begin
                    e_st <= E_HOLD;
                end else begin
                    bit_n      <= bit_n + 1'b1;
                    tx_sh      <= {tx_sh[54:0], 1'b0};
                    o_spi_sclk <= 1'b0;
                    sdio_out   <= tx_sh[54];
                    e_st       <= E_LOW;
                end
            end
        end

        E_HOLD:
            if (half_done) begin
                o_spi_cs_n <= 1'b1;
                sdio_oe    <= 1'b0;
                e_st       <= E_GAP;
            end

        E_GAP:                                  // CS high >= 150 ns between transfers
            if (half_done) begin
                e_done <= 1'b1;
                e_st   <= E_IDLE;
            end

        default: e_st <= E_IDLE;
        endcase
    end
end

//=============================================================================
// Sequencer: configure, read DEVID, then poll and read samples
//=============================================================================

// DATA_FORMAT goes first: it switches the sensor to 3-wire mode
always @(*)
    case (s_idx)
    3'd0:    cfg = {R_DATA_FORMAT, DATA_FORMAT};
    3'd1:    cfg = {R_POWER_CTL,   8'h00};       // standby while configuring
    3'd2:    cfg = {R_BW_RATE,     BW_RATE};
    3'd3:    cfg = {R_INT_ENABLE,  8'h00};       // INT pin not used
    3'd4:    cfg = {R_FIFO_CTL,    8'h00};       // FIFO bypass
    default: cfg = {R_POWER_CTL,   8'h08};       // measure
    endcase

// command byte: R/W, MB (multi-byte), 6-bit address
task spi_write(input [5:0] adr, input [7:0] val);
begin
    req_bits  <= {2'b00, adr, val, 40'd0};
    req_nbits <= 6'd16;
    req_read  <= 1'b0;
    e_start   <= 1'b1;
    s_busy    <= 1'b1;
end
endtask

task spi_read(input [5:0] adr, input [2:0] nbytes);
begin
    req_bits  <= {1'b1, nbytes > 3'd1, adr, 48'd0};
    req_nbits <= 6'd8 + {nbytes, 3'b000};
    req_read  <= 1'b1;
    e_start   <= 1'b1;
    s_busy    <= 1'b1;
end
endtask

always @(posedge i_clk)
begin
    e_start <= 1'b0;
    o_valid <= 1'b0;
    if (~&s_wait) s_wait <= s_wait + 1'b1;

    if (i_reset) begin
        s_st    <= S_START;
        s_busy  <= 1'b0;
        s_wait  <= 0;
        o_devid <= 8'h00;
    end else if (s_busy) begin
        if (e_done) s_busy <= 1'b0;             // wait for the transfer to finish
    end else begin
        case (s_st)
        S_START:
            if (s_wait >= STARTUP_CYCLES) begin
                s_idx <= 0;
                s_st  <= S_CFG;
            end

        S_CFG:
            if (s_idx == N_CFG) begin
                spi_read(R_DEVID, 3'd1);
                s_st <= S_ID;
            end else begin
                spi_write(cfg[13:8], cfg[7:0]);
                s_idx <= s_idx + 1'b1;
            end

        S_ID:
        begin
            o_devid <= rx_sh[7:0];
            s_wait  <= 0;
            spi_read(R_INT_SOURCE, 3'd1);
            s_st    <= S_POLL;
        end

        S_POLL:
            if (rx_sh[7]) begin                 // INT_SOURCE.DATA_READY
                spi_read(R_DATAX0, 3'd6);
                s_st <= S_READ;
            end else if (s_wait >= TIMEOUT_CYCLES) begin
                s_idx <= 0;
                s_st  <= S_CFG;
            end else begin
                spi_read(R_INT_SOURCE, 3'd1);
            end

        S_READ:
        begin
            // DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1 (little-endian pairs)
            o_x     <= {rx_sh[39:32], rx_sh[47:40]};
            o_y     <= {rx_sh[23:16], rx_sh[31:24]};
            o_z     <= {rx_sh[7:0],   rx_sh[15:8]};
            o_valid <= 1'b1;
            s_wait  <= 0;
            spi_read(R_INT_SOURCE, 3'd1);
            s_st    <= S_POLL;
        end

        default: s_st <= S_START;
        endcase
    end
end

endmodule
