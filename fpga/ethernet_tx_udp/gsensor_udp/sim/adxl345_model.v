// Behavioural ADXL345 for the testbench: 3-wire SPI slave, mode 3.
// - registers are written/read over SPI (multi-byte with the MB bit)
// - only drives SDIO once DATA_FORMAT.SPI (3-wire) is set, like the real part
// - while POWER_CTL.Measure is set it makes a new sample every DR_PERIOD_NS:
//   sample n gives X = 7n - 100, Y = -3n, Z = 256 + n and sets DATA_READY;
//   reading DATAZ1 (0x37) clears DATA_READY
// - flags SPI errors: CS falling with SCLK low, SCLK faster than 5 MHz,
//   SDIO driven by both sides
`timescale 1ns/1ps

module adxl345_model
#(
    parameter DR_PERIOD_NS = 100_000
)
(
    input  wire cs_n,
    input  wire sclk,
    inout  wire sdio
);

reg [7:0] regs [0:63];
reg [7:0] cmd;
reg [7:0] wsh;
integer   bitc;
integer   sample = 0;
integer   errors = 0;
reg       drive  = 0;
reg       dout   = 0;
realtime  t_edge = 0;

wire       is_read = cmd[7];
wire       mb      = cmd[6];
wire [5:0] adr     = cmd[5:0];

assign sdio = drive ? dout : 1'bz;

integer i;
initial begin
    for (i = 0; i < 64; i = i + 1) regs[i] = 8'h00;
    regs[6'h00] = 8'hE5;    // DEVID
    regs[6'h2C] = 8'h0A;    // BW_RATE reset value
end

function [5:0] byte_adr(input integer nbyte);
    byte_adr = mb ? adr + nbyte : adr;
endfunction

task check_half_period;
    begin
        if (t_edge > 0 && $realtime - t_edge < 100.0) begin
            $display("ADXL345 ERROR: SCLK half period %0.1f ns, minimum 100 ns", $realtime - t_edge);
            errors = errors + 1;
        end
        t_edge = $realtime;
    end
endtask

always @(negedge cs_n) begin
    bitc = 0;
    cmd  = 0;
    t_edge = 0;
    if (sclk !== 1'b1) begin
        $display("ADXL345 ERROR: CS fell with SCLK=%b (mode 3 needs SCLK high)", sclk);
        errors = errors + 1;
    end
end

always @(posedge sclk) if (!cs_n) begin
    check_half_period;
    if (sdio === 1'bx) begin
        $display("ADXL345 ERROR: SDIO contention at bit %0d", bitc);
        errors = errors + 1;
    end
    if (bitc < 8)
        cmd = {cmd[6:0], sdio};
    else if (!is_read) begin
        wsh = {wsh[6:0], sdio};
        if ((bitc - 8) % 8 == 7)
            regs[byte_adr((bitc - 8) / 8)] = wsh;
    end
    bitc = bitc + 1;
end

always @(negedge sclk) if (!cs_n) begin
    check_half_period;
    if (bitc >= 8 && is_read && regs[6'h31][6]) begin
        drive = 1;
        dout  = regs[byte_adr((bitc - 8) / 8)][7 - (bitc - 8) % 8];
    end
end

always @(posedge cs_n) begin
    drive = 0;
    if (is_read && bitc >= 16) begin
        // last register read in this transfer
        if (byte_adr((bitc - 8) / 8 - 1) == 6'h37) regs[6'h30] = regs[6'h30] & 8'h7F;
    end
end

// new sample every DR_PERIOD_NS while measuring; never in the middle of a transfer
reg [15:0] vx, vy, vz;
always begin
    #(DR_PERIOD_NS);
    if (regs[6'h2D][3]) begin
        if (!cs_n) begin
            @(posedge cs_n);
            #1;                             // after the read above has cleared DATA_READY
        end
        sample = sample + 1;
        vx = 7 * sample - 100;
        vy = -3 * sample;
        vz = 256 + sample;
        {regs[6'h33], regs[6'h32]} = vx;
        {regs[6'h35], regs[6'h34]} = vy;
        {regs[6'h37], regs[6'h36]} = vz;
        regs[6'h30] = regs[6'h30] | 8'h80;   // DATA_READY
    end
end

endmodule
