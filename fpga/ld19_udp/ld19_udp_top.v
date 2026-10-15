//=============================================================================
// DE0-Nano: LD19 lidar (UART 230400 8N1) -> UDP over a LAN8720 RMII PHY.
// Every LD19 packet with a good CRC (12 points, ~375 per second) is sent as
// its own 101-byte broadcast frame to 192.168.17.255:1025 (see ld19_frame.v).
// Everything runs on the PHY's 50 MHz REF_CLK.
//
// LED0 blinks while packets are sent (toggles every 64 packets)
// LED1 is on while bytes arrive from the LD19 (wiring/power check)
// LED7 lights for ~0.3 s after a packet with a bad CRC
//=============================================================================

module ld19_udp_top
#(
    parameter CLKS_PER_BIT = 217        // 50 MHz / 230400 baud
)
(
    input  wire       i_clk,            // 50 MHz REF_CLK from the PHY (nINT/RETCLK)

    output wire [1:0] o_eth_data,       // RMII TXD[1:0]
    output wire       o_eth_tx_en,      // RMII TX_EN

    input  wire       i_lidar_rx,       // LD19 pin 1 (Tx)

    output wire [7:0] o_led
);

localparam FRAME_LEN = 11'd101;

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

wire         pkt_valid;
wire [375:0] pkt;
wire [31:0]  pkt_time;
wire [15:0]  bad;

ld19_packet PACKET
(
    .i_clk        ( i_clk     ),
    .i_reset      ( reset     ),
    .i_byte_valid ( rx_valid  ),
    .i_byte       ( rx_byte   ),
    .i_time_us    ( time_us   ),
    .o_valid      ( pkt_valid ),
    .o_pkt        ( pkt       ),
    .o_time_us    ( pkt_time  ),
    .o_bad        ( bad       )
);

//=============================================================================
// The packet being sent is copied, so a new one can arrive meanwhile
reg [375:0] fpkt = 0;
reg [31:0]  ft = 0, fseq = 0, seq = 0;
reg [15:0]  fbad = 0;
reg         pending = 0;

reg         tx_en = 0;
wire        tx_ready;
reg [2:0]   tx_st = 0;
reg [6:0]   gap = 0;

wire take = (tx_st == 0) && pending && tx_ready;

always @(posedge i_clk)
begin
    if (reset) begin
        pending <= 1'b0;
        tx_en   <= 1'b0;
        tx_st   <= 0;
    end else begin
        if (pkt_valid) pending <= 1'b1;
        else if (take) pending <= 1'b0;

        tx_en <= 1'b0;
        case (tx_st)
        0:
            if (take) begin
                fpkt  <= pkt;
                ft    <= pkt_time;
                fbad  <= bad;
                fseq  <= seq;
                seq   <= seq + 1'b1;
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
wire [10:0] ram_adr;
wire [7:0]  ram_data;
wire        ram_re;

ld19_frame FRAME
(
    .i_clk     ( i_clk    ),
    .i_adr     ( ram_adr  ),
    .i_re      ( ram_re   ),
    .o_data    ( ram_data ),
    .i_seq     ( fseq     ),
    .i_time_us ( ft       ),
    .i_bad     ( fbad     ),
    .i_pkt     ( fpkt     )
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

//=============================================================================
// LEDs: byte activity (~50 ms hold) and bad CRC (~0.3 s hold)
reg [21:0] rx_hold  = 0;
reg [23:0] bad_hold = 0;
reg [15:0] bad_q    = 0;
always @(posedge i_clk)
begin
    bad_q <= bad;
    if (rx_valid)        rx_hold <= {22{1'b1}};
    else if (|rx_hold)   rx_hold <= rx_hold - 1'b1;
    if (bad != bad_q || rx_frame_err) bad_hold <= 24'd15_000_000;
    else if (|bad_hold)  bad_hold <= bad_hold - 1'b1;
end

assign o_led = {|bad_hold, 5'd0, |rx_hold, seq[5]};

endmodule
