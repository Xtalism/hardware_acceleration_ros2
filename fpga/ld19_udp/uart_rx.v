//=============================================================================
// UART receiver, 8N1, LSB first. Re-synchronises on every start bit and
// samples the middle of each bit. o_frame_err pulses when the stop bit is 0.
//=============================================================================

module uart_rx
#(
    parameter CLKS_PER_BIT = 217        // 50 MHz / 230400 baud
)
(
    input  wire       i_clk,
    input  wire       i_reset,
    input  wire       i_rx,

    output reg        o_valid,          // one-clock pulse with o_data
    output reg  [7:0] o_data,
    output reg        o_frame_err
);

localparam S_IDLE = 2'd0, S_START = 2'd1, S_DATA = 2'd2, S_STOP = 2'd3;

reg [1:0]  sync = 2'b11;                // i_rx is asynchronous
reg [1:0]  st   = S_IDLE;
reg [15:0] cnt  = 0;
reg [2:0]  bit_n;
reg [7:0]  sh;

wire rx = sync[1];

always @(posedge i_clk) sync <= {sync[0], i_rx};

always @(posedge i_clk)
begin
    o_valid     <= 1'b0;
    o_frame_err <= 1'b0;

    if (i_reset) begin
        st <= S_IDLE;
    end else
    case (st)
    S_IDLE:
        if (!rx) begin
            cnt <= 0;
            st  <= S_START;
        end

    S_START:                            // middle of the start bit: still low?
        if (cnt == CLKS_PER_BIT / 2 - 1) begin
            cnt   <= 0;
            bit_n <= 0;
            st    <= rx ? S_IDLE : S_DATA;
        end else
            cnt <= cnt + 1'b1;

    S_DATA:
        if (cnt == CLKS_PER_BIT - 1) begin
            cnt <= 0;
            sh  <= {rx, sh[7:1]};
            if (bit_n == 3'd7) st <= S_STOP;
            bit_n <= bit_n + 1'b1;
        end else
            cnt <= cnt + 1'b1;

    S_STOP:                             // middle of the stop bit
        if (cnt == CLKS_PER_BIT - 1) begin
            if (rx) begin
                o_data  <= sh;
                o_valid <= 1'b1;
            end else
                o_frame_err <= 1'b1;
            st <= S_IDLE;
        end else
            cnt <= cnt + 1'b1;

    default: st <= S_IDLE;
    endcase
end

endmodule
