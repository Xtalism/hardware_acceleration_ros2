//=============================================================================
// Finds LD19 packets in the UART byte stream (LD19 Development Manual 3.1):
//   0x54, 0x2C, speed, start angle, 12 x (distance, intensity), end angle,
//   timestamp, CRC-8   = 47 bytes
// CRC-8: polynomial 0x4D, initial 0, over the first 46 bytes (same as the
// manual's CrcTable). Packets with a good CRC come out on o_pkt (first byte in
// the top bits) with a one-clock o_valid pulse; o_time_us is when their first
// byte arrived. Packets with a bad CRC are dropped and counted in o_bad
// (this includes false starts while the receiver locks on).
//=============================================================================

module ld19_packet
(
    input  wire           i_clk,
    input  wire           i_reset,

    input  wire           i_byte_valid,
    input  wire [7:0]     i_byte,
    input  wire [31:0]    i_time_us,

    output reg            o_valid,
    output reg  [375:0]   o_pkt,
    output reg  [31:0]    o_time_us,
    output reg  [15:0]    o_bad
);

localparam PKT_LEN = 6'd47;

reg [367:0] buffer;                     // bytes 0..45 of the packet being received
reg [5:0]   n = 0;                      // bytes received so far
reg [7:0]   crc;
reg [31:0]  t_first;

function [7:0] crc8(input [7:0] c, input [7:0] d);
    integer k;
    begin
        crc8 = c ^ d;
        for (k = 0; k < 8; k = k + 1)
            crc8 = crc8[7] ? {crc8[6:0], 1'b0} ^ 8'h4D : {crc8[6:0], 1'b0};
    end
endfunction

always @(posedge i_clk)
begin
    o_valid <= 1'b0;

    if (i_reset) begin
        n     <= 0;
        o_bad <= 0;
    end else if (i_byte_valid) begin
        if (n == 0 || (n == 1 && i_byte != 8'h2C)) begin
            // hunting for the header (a lone 0x54 can also restart it)
            if (i_byte == 8'h54) begin
                buffer  <= {buffer[359:0], i_byte};
                crc     <= crc8(8'h00, i_byte);
                t_first <= i_time_us;
                n       <= 1;
            end else
                n <= 0;
        end else if (n == PKT_LEN - 1) begin
            // last byte is the CRC
            if (i_byte == crc) begin
                o_pkt     <= {buffer, i_byte};
                o_time_us <= t_first;
                o_valid   <= 1'b1;
            end else
                o_bad <= o_bad + 1'b1;
            n <= 0;
        end else begin
            buffer <= {buffer[359:0], i_byte};
            crc    <= crc8(crc, i_byte);
            n      <= n + 1'b1;
        end
    end
end

endmodule
