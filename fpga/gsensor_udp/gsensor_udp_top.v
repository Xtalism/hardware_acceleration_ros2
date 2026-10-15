//=============================================================================
// DE0-Nano: ADXL345 samples -> UDP over a LAN8720 RMII PHY.
// Every sample (400 Hz by default) is sent as its own 60-byte broadcast
// frame to 192.168.17.255:1024 (see gsensor_frame.v for the payload).
// Everything runs on the PHY's 50 MHz REF_CLK.
//
// LED0 blinks while samples are sent (toggles every 128 packets),
// LED1 is on when the ADXL345 answered with DEVID 0xE5.
//=============================================================================

module gsensor_udp_top
#(
    parameter       STARTUP_CYCLES = 500_000,   // 10 ms before talking to the ADXL345
    parameter [7:0] BW_RATE        = 8'h0C      // ADXL345 output data rate: 400 Hz
)
(
    input  wire       i_clk,          // 50 MHz REF_CLK from the PHY (nINT/RETCLK)

    output wire [1:0] o_eth_data,     // RMII TXD[1:0]
    output wire       o_eth_tx_en,    // RMII TX_EN

    output wire       o_g_cs_n,       // G_SENSOR_CS_N
    output wire       o_g_sclk,       // I2C_SCLK
    inout  wire       io_g_sdio,      // I2C_SDAT

    output wire [7:0] o_led
);

localparam FRAME_LEN = 11'd60;

// power-on reset
reg [3:0] rst_cnt = 0;
wire      reset = ~&rst_cnt;
always @(posedge i_clk) if (reset) rst_cnt <= rst_cnt + 1'b1;

// microsecond timestamp
reg [5:0]  us_div = 0;
reg [31:0] time_us = 0;
always @(posedge i_clk)
    if (us_div == 6'd49) begin
        us_div  <= 0;
        time_us <= time_us + 1'b1;
    end else
        us_div <= us_div + 1'b1;

//=============================================================================
wire        s_valid;
wire [15:0] s_x, s_y, s_z;
wire [7:0]  devid;

adxl345_spi3
#(
    .STARTUP_CYCLES ( STARTUP_CYCLES ),
    .BW_RATE        ( BW_RATE        )
)
GSENSOR
(
    .i_clk       ( i_clk     ),
    .i_reset     ( reset     ),
    .o_spi_cs_n  ( o_g_cs_n  ),
    .o_spi_sclk  ( o_g_sclk  ),
    .io_spi_sdio ( io_g_sdio ),
    .o_valid     ( s_valid   ),
    .o_x         ( s_x       ),
    .o_y         ( s_y       ),
    .o_z         ( s_z       ),
    .o_devid     ( devid     )
);

//=============================================================================
// Latest sample, and the copy that is being sent
reg [15:0] lx, ly, lz;
reg [31:0] lt;
reg        pending = 0;
reg        dropped = 0;

reg [15:0] fx = 0, fy = 0, fz = 0;
reg [31:0] ft = 0, fseq = 0;
reg [7:0]  fflags = 0, fdevid = 0;
reg [31:0] seq = 0;

reg        tx_en = 0;
wire       tx_ready;
reg [2:0]  tx_st = 0;
reg [6:0]  gap = 0;

wire take = (tx_st == 0) && pending && tx_ready;

always @(posedge i_clk)
begin
    if (s_valid) begin
        lx <= s_x;
        ly <= s_y;
        lz <= s_z;
        lt <= time_us;
    end

    if (reset) begin
        pending <= 1'b0;
        dropped <= 1'b0;
        tx_en   <= 1'b0;
        tx_st   <= 0;
    end else begin
        if (s_valid)   pending <= 1'b1;
        else if (take) pending <= 1'b0;

        if (take)                             dropped <= 1'b0;
        else if (s_valid && pending)          dropped <= 1'b1;

        tx_en <= 1'b0;
        case (tx_st)
        0:
            if (take) begin
                fx     <= lx;
                fy     <= ly;
                fz     <= lz;
                ft     <= lt;
                fseq   <= seq;
                fflags <= {7'd0, dropped};
                fdevid <= devid;
                seq    <= seq + 1'b1;
                tx_en  <= 1'b1;
                tx_st  <= 1;
            end
        1: tx_st <= 2;                          // eth_tx drops tx_ready
        2: tx_st <= 3;
        3:
            if (tx_ready) begin
                gap   <= 0;
                tx_st <= 4;
            end
        4:                                      // inter-frame gap >= 96 bit times
        begin
            gap <= gap + 1'b1;
            if (&gap) tx_st <= 0;
        end
        default: tx_st <= 0;
        endcase
    end
end

//=============================================================================
wire [10:0] ram_adr;
wire [7:0]  ram_data;
wire        ram_re;

gsensor_frame FRAME
(
    .i_clk     ( i_clk    ),
    .i_adr     ( ram_adr  ),
    .i_re      ( ram_re   ),
    .o_data    ( ram_data ),
    .i_seq     ( fseq     ),
    .i_time_us ( ft       ),
    .i_x       ( fx       ),
    .i_y       ( fy       ),
    .i_z       ( fz       ),
    .i_devid   ( fdevid   ),
    .i_flags   ( fflags   )
);

eth_tx
#(
    .PAKET_MAX_SIZE( 11'd1500 )
)
ETH_TX
(
    .i_clk           ( i_clk       ),
    .i_reset         ( reset       ),
    .i_tx_en         ( tx_en       ),
    .o_tx_ready      ( tx_ready    ),
    .o_eth_data      ( o_eth_data  ),
    .o_eth_tx_en     ( o_eth_tx_en ),
    .i_ram_data      ( ram_data    ),
    .o_ram_adr       ( ram_adr     ),
    .o_ram_re        ( ram_re      ),
    .i_ram_data_size ( FRAME_LEN   )
);

assign o_led = {6'd0, devid == 8'hE5, seq[7]};

endmodule
