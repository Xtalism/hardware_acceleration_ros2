//=============================================================================
// Quadrature (A/B) encoder decoder, x4: counts every edge of both channels.
//
// A and B are asynchronous: each is synchronised and must then stay at its
// new level for FILTER_CYCLES clocks before the change is accepted, which
// rejects short spikes induced by the motor's PWM.
//
// A leads B (A rises first) counts up:   AB 00 -> 10 -> 11 -> 01 -> 00
// B leads A counts down.
// A step where A and B both changed at once can't be decoded (an edge was
// missed or it was noise); it is ignored and counted in o_errors (wraps).
//=============================================================================

module quad_decoder
#(
    parameter FILTER_CYCLES = 50        // 1 us at 50 MHz
)
(
    input  wire               i_clk,
    input  wire               i_reset,
    input  wire               i_a,
    input  wire               i_b,

    output reg  signed [31:0] o_count,
    output reg         [7:0]  o_errors
);

reg [1:0] a_sync = 2'b00, b_sync = 2'b00;
reg [7:0] a_cnt = 0, b_cnt = 0;
reg       a_f = 0, b_f = 0;             // filtered A and B
reg [1:0] prev = 2'b00;

always @(posedge i_clk)
begin
    a_sync <= {a_sync[0], i_a};
    b_sync <= {b_sync[0], i_b};

    if (a_sync[1] == a_f)               a_cnt <= 0;
    else if (a_cnt == FILTER_CYCLES - 1) begin
        a_f   <= a_sync[1];
        a_cnt <= 0;
    end else                            a_cnt <= a_cnt + 1'b1;

    if (b_sync[1] == b_f)               b_cnt <= 0;
    else if (b_cnt == FILTER_CYCLES - 1) begin
        b_f   <= b_sync[1];
        b_cnt <= 0;
    end else                            b_cnt <= b_cnt + 1'b1;
end

always @(posedge i_clk)
begin
    if (i_reset) begin
        o_count  <= 0;
        o_errors <= 0;
        prev     <= {a_f, b_f};
    end else begin
        prev <= {a_f, b_f};
        case ({prev, a_f, b_f})
        4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: o_count  <= o_count + 1;
        4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: o_count  <= o_count - 1;
        4'b00_11, 4'b11_00, 4'b01_10, 4'b10_01: o_errors <= o_errors + 1'b1;
        default: ;                      // no change
        endcase
    end
end

endmodule
