//=============================================================================
// One G-sensor sample as an Ethernet/IPv4/UDP frame (60 bytes, no FCS).
// eth_tx reads it like ram_ip_paket_tx: address and data both registered,
// so o_data lags i_adr by 2 clocks. The i_* payload inputs must not change
// while a frame is being sent.
//
// UDP payload, 18 bytes, big-endian:
//    0..1   "GS"
//    2..5   sequence number
//    6..9   FPGA time of the sample in microseconds
//   10..15  X, Y, Z: signed ADXL345 counts (3.9 mg/LSB in full resolution)
//   16      ADXL345 DEVID (0xE5 when the sensor answers)
//   17      flags: bit0 = at least one sample was dropped before this one
//
// UDP checksum is 0 (not used, allowed for IPv4); the IP header checksum is
// computed here from the parameters.
//=============================================================================

module gsensor_frame
#(
    parameter [47:0] DST_MAC  = 48'hFF_FF_FF_FF_FF_FF,
    parameter [47:0] SRC_MAC  = 48'h00_12_34_56_78_90,
    parameter [31:0] SRC_IP   = {8'd192, 8'd168, 8'd17, 8'd200},
    parameter [31:0] DST_IP   = {8'd192, 8'd168, 8'd17, 8'd255},
    parameter [15:0] SRC_PORT = 16'd1024,
    parameter [15:0] DST_PORT = 16'd1024
)
(
    input  wire        i_clk,
    input  wire [10:0] i_adr,
    input  wire        i_re,
    output reg  [7:0]  o_data,

    input  wire [31:0] i_seq,
    input  wire [31:0] i_time_us,
    input  wire [15:0] i_x,
    input  wire [15:0] i_y,
    input  wire [15:0] i_z,
    input  wire [7:0]  i_devid,
    input  wire [7:0]  i_flags
);

localparam FRAME_LEN = 60;
localparam [15:0] UDP_LEN = 16'd26;            // 8 + 18
localparam [15:0] IP_LEN  = 16'd46;            // 20 + UDP_LEN
localparam [15:0] IP_ID   = 16'h0000;
localparam [15:0] IP_FLAG = 16'h4000;          // don't fragment
localparam [7:0]  IP_TTL  = 8'd64;

localparam [31:0] CSUM0 = 32'h4500 + IP_LEN + IP_ID + IP_FLAG + {IP_TTL, 8'h11}
                        + SRC_IP[31:16] + SRC_IP[15:0] + DST_IP[31:16] + DST_IP[15:0];
localparam [31:0] CSUM1 = CSUM0[15:0] + CSUM0[31:16];
localparam [15:0] IP_CSUM = ~(CSUM1[15:0] + CSUM1[31:16]);

wire [FRAME_LEN*8-1:0] frame = {
    DST_MAC, SRC_MAC, 16'h0800,                                         // Ethernet
    16'h4500, IP_LEN, IP_ID, IP_FLAG, IP_TTL, 8'h11, IP_CSUM, SRC_IP, DST_IP,  // IPv4
    SRC_PORT, DST_PORT, UDP_LEN, 16'h0000,                              // UDP
    16'h4753, i_seq, i_time_us, i_x, i_y, i_z, i_devid, i_flags         // payload
};

reg [10:0] adr_q = 0;

always @(posedge i_clk)
begin
    if (i_re) adr_q <= i_adr;
    o_data <= (adr_q < FRAME_LEN) ? frame[(FRAME_LEN - 1 - adr_q) * 8 +: 8] : 8'h00;
end

endmodule
