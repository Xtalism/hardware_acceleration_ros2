//=============================================================================
// DE0-Nano: 4 wheel encoders (quadrature A/B) -> UDP over a LAN8720 RMII PHY.
// Every ENC_PERIOD_US (200 Hz) the four counts are latched together and sent
// as one 72-byte broadcast frame to 192.168.17.255:1026 (see enc_frame.v).
// Everything runs on the PHY's 50 MHz REF_CLK.
//
// LED0 blinks while packets are sent (toggles every 64 packets)
// LED1..LED4 light while encoder 0..3 is turning (count changed in the last ~0.1 s)
// LED7 lights for ~0.3 s after an undecodable step on any encoder
//=============================================================================

module encoder_udp_top
#(
    parameter ENC_PERIOD_US = 5000,     // counts sent every 5 ms (200 Hz)
    parameter ENC_FILTER    = 50        // A/B must be stable 1 us
)
(
    input  wire       i_clk,            // 50 MHz REF_CLK from the PHY (nINT/RETCLK)

    output wire [1:0] o_eth_data,       // RMII TXD[1:0]
    output wire       o_eth_tx_en,      // RMII TX_EN

    input  wire [3:0] i_enc_a,          // wheel encoders 0..3, channel A
    input  wire [3:0] i_enc_b,          //                      channel B

    output wire [7:0] o_led
);

localparam EN_LEN = 11'd72;

// power-on reset
reg [3:0] rst_cnt = 0;
wire      reset = ~&rst_cnt;
always @(posedge i_clk) if (reset) rst_cnt <= rst_cnt + 1'b1;

// microsecond timestamp
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

// counts latched together every ENC_PERIOD_US
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

//=============================================================================
// The latched counts are copied for the frame being sent
reg [127:0] en_fcounts = 0;
reg [31:0]  en_ferrors = 0, en_ft = 0, en_fseq = 0;
reg         en_pending = 0;

reg         tx_en = 0;
wire        tx_ready;
reg [2:0]   tx_st = 0;
reg [6:0]   gap = 0;

wire take = (tx_st == 0) && en_pending && tx_ready;

always @(posedge i_clk)
begin
    if (reset) begin
        en_pending <= 1'b0;
        tx_en      <= 1'b0;
        tx_st      <= 0;
    end else begin
        if (en_latch)  en_pending <= 1'b1;
        else if (take) en_pending <= 1'b0;

        tx_en <= 1'b0;
        case (tx_st)
        0:
            if (take) begin
                en_fcounts <= en_lcounts;
                en_ferrors <= en_lerrors;
                en_ft      <= en_lt;
                en_fseq    <= en_lseq;
                tx_en      <= 1'b1;
                tx_st      <= 1;
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
wire        ram_re;
wire [7:0]  en_data;

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
    .i_ram_data      ( en_data     ),
    .o_ram_adr       ( ram_adr     ),
    .o_ram_re        ( ram_re      ),
    .i_ram_data_size ( EN_LEN      )
);

//=============================================================================
// LEDs: per-encoder movement (~0.1 s hold) and decode errors (~0.3 s hold)
reg [31:0] count_q [0:3];
reg [7:0]  err_q   [0:3];
reg [22:0] move_hold [0:3];
reg [23:0] err_hold = 0;
reg [3:0]  moving = 0;
integer    i;

initial for (i = 0; i < 4; i = i + 1) begin   // power-up values
    count_q[i]   = 0;
    err_q[i]     = 0;
    move_hold[i] = 0;
end

always @(posedge i_clk)
begin
    for (i = 0; i < 4; i = i + 1) begin
        count_q[i] <= enc_count[i];
        err_q[i]   <= enc_err[i];
        if (enc_count[i] != count_q[i]) move_hold[i] <= 23'd5_000_000;
        else if (|move_hold[i])          move_hold[i] <= move_hold[i] - 1'b1;
        moving[i] <= |move_hold[i];
    end
    if (enc_err[0] != err_q[0] || enc_err[1] != err_q[1] || enc_err[2] != err_q[2] || enc_err[3] != err_q[3])
        err_hold <= 24'd15_000_000;
    else if (|err_hold)
        err_hold <= err_hold - 1'b1;
end

assign o_led = {|err_hold, 2'd0, moving, en_seq[6]};

endmodule
