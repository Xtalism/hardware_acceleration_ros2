//=============================================================================
// DE0-Nano: ADXL345 G-sensor + LD19 lidar + 4 wheel encoders -> UDP over a
// LAN8720 RMII PHY. Same packets as the single-sensor designs, so the same
// PC tools work:
//   G-sensor: every sample (~378 Hz), 60-byte frame to 192.168.17.255:1024
//             (gsensor_frame.v, from fpga/gsensor_udp)
//   LD19:     every packet with a good CRC (12 points, ~415 Hz), 101-byte
//             frame to 192.168.17.255:1025 (ld19_frame.v, from fpga/ld19_udp)
//   Encoders: the four counts latched together every ENC_PERIOD_US (200 Hz),
//             72-byte frame to 192.168.17.255:1026 (quad_decoder.v and
//             enc_frame.v, from fpga/encoder_udp)
// All are timestamped by the same microsecond counter, so their times can be
// compared directly. When several have a packet waiting they take turns.
// Everything runs on the PHY's 50 MHz REF_CLK.
//
// LED0 blinks while G-sensor samples are sent   LED1 ADXL345 answered (DEVID 0xE5)
// LED2 blinks while lidar packets are sent      LED3 bytes arriving from the LD19
// LED4..LED6 show encoder 0's count bits 9..7 (they change as wheel 0 turns)
// LED7 lights for ~0.3 s after an LD19 packet with a bad CRC
//=============================================================================

module sensors_udp_top
#(
    parameter       STARTUP_CYCLES = 500_000,   // 10 ms before talking to the ADXL345
    parameter [7:0] BW_RATE        = 8'h0C,     // ADXL345 output data rate: 400 Hz
    parameter       CLKS_PER_BIT   = 217,       // LD19 UART: 50 MHz / 230400 baud
    parameter       ENC_PERIOD_US  = 5000,      // encoder counts sent every 5 ms (200 Hz)
    parameter       ENC_FILTER     = 50         // encoder A/B must be stable 1 us
)
(
    input  wire       i_clk,          // 50 MHz REF_CLK from the PHY (nINT/RETCLK)

    output wire [1:0] o_eth_data,     // RMII TXD[1:0]
    output wire       o_eth_tx_en,    // RMII TX_EN

    output wire       o_g_cs_n,       // G_SENSOR_CS_N
    output wire       o_g_sclk,       // I2C_SCLK
    inout  wire       io_g_sdio,      // I2C_SDAT

    input  wire       i_lidar_rx,     // LD19 pin 1 (Tx)

    input  wire [3:0] i_enc_a,        // wheel encoders 0..3, channel A
    input  wire [3:0] i_enc_b,        //                      channel B

    output wire [7:0] o_led
);

localparam GS_LEN = 11'd60;
localparam LD_LEN = 11'd101;
localparam EN_LEN = 11'd72;

localparam SRC_GS = 2'd0, SRC_LD = 2'd1, SRC_EN = 2'd2;

// power-on reset
reg [3:0] rst_cnt = 0;
wire      reset = ~&rst_cnt;
always @(posedge i_clk) if (reset) rst_cnt <= rst_cnt + 1'b1;

// microsecond timestamp shared by all sensors
reg [5:0]  us_div = 0;
reg [31:0] time_us = 0;
wire       us_tick = (us_div == 6'd49);
always @(posedge i_clk)
    if (us_tick) begin
        us_div  <= 0;
        time_us <= time_us + 1'b1;
    end else
        us_div <= us_div + 1'b1;

//=============================================================================
// Sensors
wire        gs_valid;
wire [15:0] gs_x, gs_y, gs_z;
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
    .o_valid     ( gs_valid  ),
    .o_x         ( gs_x      ),
    .o_y         ( gs_y      ),
    .o_z         ( gs_z      ),
    .o_devid     ( devid     )
);

wire        rx_valid, rx_frame_err;
wire [7:0]  rx_byte;

uart_rx #( .CLKS_PER_BIT(CLKS_PER_BIT) ) UART
(
    .i_clk       ( i_clk        ),
    .i_reset     ( reset        ),
    .i_rx        ( i_lidar_rx   ),
    .o_valid     ( rx_valid     ),
    .o_data      ( rx_byte      ),
    .o_frame_err ( rx_frame_err )
);

wire         ld_valid;
wire [375:0] ld_pkt;
wire [31:0]  ld_time;
wire [15:0]  ld_bad;

ld19_packet PACKET
(
    .i_clk        ( i_clk    ),
    .i_reset      ( reset    ),
    .i_byte_valid ( rx_valid ),
    .i_byte       ( rx_byte  ),
    .i_time_us    ( time_us  ),
    .o_valid      ( ld_valid ),
    .o_pkt        ( ld_pkt   ),
    .o_time_us    ( ld_time  ),
    .o_bad        ( ld_bad   )
);

wire signed [31:0] enc_count [0:3];
wire [7:0]         enc_err   [0:3];

genvar w;
generate
    for (w = 0; w < 4; w = w + 1) begin : ENC
        quad_decoder #( .FILTER_CYCLES(ENC_FILTER) ) DEC
        (
            .i_clk    ( i_clk         ),
            .i_reset  ( reset         ),
            .i_a      ( i_enc_a[w]    ),
            .i_b      ( i_enc_b[w]    ),
            .o_count  ( enc_count[w]  ),
            .o_errors ( enc_err[w]    )
        );
    end
endgenerate

//=============================================================================
// Latest G-sensor sample (the LD19 packet is held in ld19_packet)
reg [15:0] lx, ly, lz;
reg [31:0] lt;

always @(posedge i_clk)
    if (gs_valid) begin
        lx <= gs_x;
        ly <= gs_y;
        lz <= gs_z;
        lt <= time_us;
    end

// Encoder counts latched together every ENC_PERIOD_US
reg [15:0]  enc_us = 0;
reg [127:0] en_lcounts = 0;
reg [31:0]  en_lerrors = 0;
reg [31:0]  en_lt = 0, en_lseq = 0, en_seq = 0;
reg         en_latch = 0;

always @(posedge i_clk)
begin
    en_latch <= 1'b0;
    if (reset)
        enc_us <= 0;
    else if (us_tick) begin
        if (enc_us == ENC_PERIOD_US - 1) begin
            enc_us     <= 0;
            en_lcounts <= {enc_count[0], enc_count[1], enc_count[2], enc_count[3]};
            en_lerrors <= {enc_err[0], enc_err[1], enc_err[2], enc_err[3]};
            en_lt      <= time_us + 1'b1;       // the microsecond that starts now
            en_lseq    <= en_seq;
            en_seq     <= en_seq + 1'b1;
            en_latch   <= 1'b1;
        end else
            enc_us <= enc_us + 1'b1;
    end
end

// Copies for the frame being sent, so new data can arrive meanwhile
reg [15:0]  fx = 0, fy = 0, fz = 0;
reg [31:0]  gs_ft = 0, gs_fseq = 0, gs_seq = 0;
reg [7:0]   gs_fflags = 0, gs_fdevid = 0;
reg [375:0] ld_fpkt = 0;
reg [31:0]  ld_ft = 0, ld_fseq = 0, ld_seq = 0;
reg [15:0]  ld_fbad = 0;
reg [127:0] en_fcounts = 0;
reg [31:0]  en_ferrors = 0, en_ft = 0, en_fseq = 0;

reg         gs_pending = 0, ld_pending = 0, en_pending = 0, gs_dropped = 0;
reg [1:0]   last = SRC_EN;              // who was sent last, for taking turns
reg [1:0]   sel  = SRC_GS;              // frame source while sending
reg [10:0]  tx_len = GS_LEN;

reg         tx_en = 0;
wire        tx_ready;
reg [2:0]   tx_st = 0;
reg [6:0]   gap = 0;

// round robin: the first waiting source after the one sent last
wire [2:0] pend = {en_pending, ld_pending, gs_pending};
reg  [1:0] pick;
always @(*)
    case (last)
    SRC_GS:  pick = pend[SRC_LD] ? SRC_LD : pend[SRC_EN] ? SRC_EN : SRC_GS;
    SRC_LD:  pick = pend[SRC_EN] ? SRC_EN : pend[SRC_GS] ? SRC_GS : SRC_LD;
    default: pick = pend[SRC_GS] ? SRC_GS : pend[SRC_LD] ? SRC_LD : SRC_EN;
    endcase

wire start   = (tx_st == 0) && tx_ready && (|pend);
wire take_gs = start && pick == SRC_GS;
wire take_ld = start && pick == SRC_LD;
wire take_en = start && pick == SRC_EN;

always @(posedge i_clk)
begin
    if (reset) begin
        gs_pending <= 1'b0;
        ld_pending <= 1'b0;
        en_pending <= 1'b0;
        gs_dropped <= 1'b0;
        tx_en      <= 1'b0;
        tx_st      <= 0;
    end else begin
        if (gs_valid)     gs_pending <= 1'b1;
        else if (take_gs) gs_pending <= 1'b0;
        if (ld_valid)     ld_pending <= 1'b1;
        else if (take_ld) ld_pending <= 1'b0;
        if (en_latch)     en_pending <= 1'b1;
        else if (take_en) en_pending <= 1'b0;

        if (take_gs)                     gs_dropped <= 1'b0;
        else if (gs_valid && gs_pending) gs_dropped <= 1'b1;

        tx_en <= 1'b0;
        case (tx_st)
        0:
            if (start) begin
                case (pick)
                SRC_GS: begin
                    fx        <= lx;
                    fy        <= ly;
                    fz        <= lz;
                    gs_ft     <= lt;
                    gs_fseq   <= gs_seq;
                    gs_fflags <= {7'd0, gs_dropped};
                    gs_fdevid <= devid;
                    gs_seq    <= gs_seq + 1'b1;
                    tx_len    <= GS_LEN;
                end
                SRC_LD: begin
                    ld_fpkt   <= ld_pkt;
                    ld_ft     <= ld_time;
                    ld_fbad   <= ld_bad;
                    ld_fseq   <= ld_seq;
                    ld_seq    <= ld_seq + 1'b1;
                    tx_len    <= LD_LEN;
                end
                default: begin
                    en_fcounts <= en_lcounts;
                    en_ferrors <= en_lerrors;
                    en_ft      <= en_lt;
                    en_fseq    <= en_lseq;
                    tx_len     <= EN_LEN;
                end
                endcase
                sel   <= pick;
                last  <= pick;
                tx_en <= 1'b1;
                tx_st <= 1;
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
// The frame builders follow eth_tx's read address; sel picks the one sent
wire [10:0] ram_adr;
wire        ram_re;
wire [7:0]  gs_data, ld_data, en_data;

gsensor_frame GS_FRAME
(
    .i_clk     ( i_clk     ),
    .i_adr     ( ram_adr   ),
    .i_re      ( ram_re    ),
    .o_data    ( gs_data   ),
    .i_seq     ( gs_fseq   ),
    .i_time_us ( gs_ft     ),
    .i_x       ( fx        ),
    .i_y       ( fy        ),
    .i_z       ( fz        ),
    .i_devid   ( gs_fdevid ),
    .i_flags   ( gs_fflags )
);

ld19_frame LD_FRAME
(
    .i_clk     ( i_clk    ),
    .i_adr     ( ram_adr  ),
    .i_re      ( ram_re   ),
    .o_data    ( ld_data  ),
    .i_seq     ( ld_fseq  ),
    .i_time_us ( ld_ft    ),
    .i_bad     ( ld_fbad  ),
    .i_pkt     ( ld_fpkt  )
);

enc_frame EN_FRAME
(
    .i_clk     ( i_clk      ),
    .i_adr     ( ram_adr    ),
    .i_re      ( ram_re     ),
    .o_data    ( en_data    ),
    .i_seq     ( en_fseq    ),
    .i_time_us ( en_ft      ),
    .i_counts  ( en_fcounts ),
    .i_errors  ( en_ferrors )
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
    .i_ram_data      ( sel == SRC_GS ? gs_data : sel == SRC_LD ? ld_data : en_data ),
    .o_ram_adr       ( ram_adr     ),
    .o_ram_re        ( ram_re      ),
    .i_ram_data_size ( tx_len      )
);

//=============================================================================
// LEDs: LD19 byte activity (~50 ms hold) and bad CRC (~0.3 s hold)
reg [21:0] rx_hold  = 0;
reg [23:0] bad_hold = 0;
reg [15:0] bad_q    = 0;
always @(posedge i_clk)
begin
    bad_q <= ld_bad;
    if (rx_valid)                        rx_hold <= {22{1'b1}};
    else if (|rx_hold)                   rx_hold <= rx_hold - 1'b1;
    if (ld_bad != bad_q || rx_frame_err) bad_hold <= 24'd15_000_000;
    else if (|bad_hold)                  bad_hold <= bad_hold - 1'b1;
end

assign o_led = {|bad_hold, enc_count[0][9:7], |rx_hold, ld_seq[5], devid == 8'hE5, gs_seq[7]};

endmodule
