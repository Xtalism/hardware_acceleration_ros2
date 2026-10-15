// Self-checking testbench for the wheel encoders, run against
//   encoder_udp_top   (default: fpga/encoder_udp/sim/run_sim.sh)
//   sensors_udp_top   (with -DSENSORS_TOP: fpga/sensors_udp/sim/run_sim.sh;
//                      its ADXL345 and LD19 inputs are left unconnected)
// Four behavioural quadrature encoders drive A/B (A leads B = forward):
//   wheel 0: forward, slow then very fast (a step every 3 us)
//   wheel 1: backwards
//   wheel 2: forward with a 0.5 us spike on A before every step (must be filtered out:
//            checked by counting every change of the decoder's count)
//   wheel 3: forward with two impossible steps where A and B change together
//            (must be counted as errors and not change the count)
// Every encoder frame (UDP 1026) is checked: header == expected_en_hdr.hex,
// "EN", sequence 0,1,2..., time exactly ENC_PERIOD_US apart, and the four
// counts and error counts equal what the encoders had done when the FPGA
// latched them. Steps are kept >= 10 us away from the latch moments so the
// expected value is unambiguous. Prints PASS or FAIL.

`timescale 1ns/1ps

module tb_encoder_udp;

localparam PERIOD_US = 200;
localparam N_FRAMES  = 30;

reg clk = 0;
always #10 clk = ~clk;                      // 50 MHz REF_CLK

reg  [3:0] enc_a = 4'b0000, enc_b = 4'b0000;
wire [1:0] txd;
wire       tx_en;
wire [7:0] led;

`ifdef SENSORS_TOP
wire       cs_n, sclk, sdio;
pullup (sdio);                              // no ADXL345: it just never answers

sensors_udp_top #( .STARTUP_CYCLES(1000), .ENC_PERIOD_US(PERIOD_US) ) dut
(
    .i_clk       ( clk   ),
    .o_eth_data  ( txd   ),
    .o_eth_tx_en ( tx_en ),
    .o_g_cs_n    ( cs_n  ),
    .o_g_sclk    ( sclk  ),
    .io_g_sdio   ( sdio  ),
    .i_lidar_rx  ( 1'b1  ),
    .i_enc_a     ( enc_a ),
    .i_enc_b     ( enc_b ),
    .o_led       ( led   )
);
`else
encoder_udp_top #( .ENC_PERIOD_US(PERIOD_US) ) dut
(
    .i_clk       ( clk   ),
    .o_eth_data  ( txd   ),
    .o_eth_tx_en ( tx_en ),
    .i_enc_a     ( enc_a ),
    .i_enc_b     ( enc_b ),
    .o_led       ( led   )
);
`endif

//=============================================================================
// Encoder models
integer phase [0:3];
integer exp_count [0:3];
integer exp_err [0:3];
integer w;

initial for (w = 0; w < 4; w = w + 1) begin
    phase[w] = 0;
    exp_count[w] = 0;
    exp_err[w] = 0;
end

task automatic drive(input integer wh);    // AB for the phase: 00, 10, 11, 01
    begin
        enc_a[wh] = (phase[wh] == 1 || phase[wh] == 2);
        enc_b[wh] = (phase[wh] == 2 || phase[wh] == 3);
    end
endtask

task automatic wait_window;                 // stay away from the latch moments
    while (($time % (PERIOD_US * 1000)) < 10_000 || ($time % (PERIOD_US * 1000)) > PERIOD_US * 1000 - 15_000)
        #1000;
endtask

task automatic step(input integer wh, input integer dir);   // automatic: called by all four wheels at once
    begin
        wait_window;
        phase[wh] = (phase[wh] + dir + 4) % 4;
        drive(wh);
        exp_count[wh] = exp_count[wh] + dir;
    end
endtask

integer k;
initial begin                               // wheel 0
    #50_000;
    for (k = 0; k < 300; k = k + 1) begin step(0, 1); #7_000; end
    for (k = 0; k < 600; k = k + 1) begin step(0, 1); #3_000; end
end

initial begin : wheel1
    integer j;
    #50_000;
    for (j = 0; j < 250; j = j + 1) begin step(1, -1); #9_000; end
end

initial begin : wheel2
    integer j;
    #50_000;
    for (j = 0; j < 150; j = j + 1) begin
        wait_window;
        enc_a[2] = ~enc_a[2];               // 0.5 us spike
        #500;
        enc_a[2] = ~enc_a[2];
        #3_000;
        step(2, 1);
        #8_000;
    end
end

initial begin : wheel3
    integer j;
    #50_000;
    for (j = 0; j < 100; j = j + 1) begin
        if (j == 30 || j == 60) begin       // A and B change together: undecodable
            wait_window;
            phase[3] = (phase[3] + 2) % 4;
            drive(3);
            exp_err[3] = exp_err[3] + 1;
        end else
            step(3, 1);
        #13_000;
    end
end

// every change of each decoder's count: one per real step, so a spike that got
// through the filter (one step forward and back, invisible in the packets) shows here
integer changes [0:3];
initial for (w = 0; w < 4; w = w + 1) changes[w] = 0;
always @(dut.ENC[0].DEC.o_count) if ($time > 1000) changes[0] = changes[0] + 1;   // not the reset
always @(dut.ENC[1].DEC.o_count) if ($time > 1000) changes[1] = changes[1] + 1;   // not the reset
always @(dut.ENC[2].DEC.o_count) if ($time > 1000) changes[2] = changes[2] + 1;   // not the reset
always @(dut.ENC[3].DEC.o_count) if ($time > 1000) changes[3] = changes[3] + 1;   // not the reset

// what the encoders had done when the FPGA latched (en_latch is one clock later)
integer snap_count [0:4*256-1];
integer snap_err   [0:4*256-1];
always @(posedge clk)
    if (dut.en_latch && dut.en_lseq < 256)
        for (w = 0; w < 4; w = w + 1) begin
            snap_count[dut.en_lseq * 4 + w] = exp_count[w];
            snap_err[dut.en_lseq * 4 + w]   = exp_err[w];
        end

//=============================================================================
// Frame checker
reg [7:0]  en_hdr [0:71];
reg [7:0]  rx_buf [0:2047];
integer    rx_len, dibits, frames = 0, en_frames = 0, errors = 0, gap = 0;
reg [7:0]  sh;
reg        tx_en_d = 0;
reg [31:0] t_prev = 0;

initial $readmemh("expected_en_hdr.hex", en_hdr);

function [31:0] crc32_byte(input [31:0] crc, input [7:0] d);
    integer i;
    begin
        crc32_byte = crc ^ d;
        for (i = 0; i < 8; i = i + 1)
            crc32_byte = crc32_byte[0] ? (crc32_byte >> 1) ^ 32'hEDB88320 : (crc32_byte >> 1);
    end
endfunction

function [31:0] be32(input integer off);
    be32 = {rx_buf[8 + off], rx_buf[9 + off], rx_buf[10 + off], rx_buf[11 + off]};
endfunction

task chk(input [255:0] what, input [31:0] got, input [31:0] want);
    if (got !== want) begin
        $display("ERROR encoder frame %0d: %0s = %0d, expected %0d", en_frames, what, $signed(got), $signed(want));
        errors = errors + 1;
    end
endtask

task check_frame;
    integer    i, flen, bad, seq;
    reg [31:0] crc;
    begin
        frames = frames + 1;
        flen   = rx_len - 12;
        crc = 32'hFFFFFFFF;
        for (i = 0; i < flen; i = i + 1) crc = crc32_byte(crc, rx_buf[8 + i]);
        chk("FCS ok", {rx_buf[8 + flen + 3], rx_buf[8 + flen + 2], rx_buf[8 + flen + 1], rx_buf[8 + flen]} == ~crc, 1);

        if ({rx_buf[8 + 36], rx_buf[8 + 37]} == 16'd1026) begin
            bad = 0;
            for (i = 0; i < 42; i = i + 1) if (rx_buf[8 + i] !== en_hdr[i]) bad = bad + 1;
            chk("header bytes wrong", bad, 0);
            chk("frame length", flen, 72);
            chk("magic", {rx_buf[8 + 42], rx_buf[8 + 43]}, 16'h454E);
            seq = be32(44);
            chk("sequence", seq, en_frames);
            if (en_frames > 0) chk("time step (us)", be32(48) - t_prev, PERIOD_US);
            t_prev = be32(48);
            for (i = 0; i < 4; i = i + 1) begin
                chk("count", be32(52 + 4 * i), snap_count[seq * 4 + i]);
                chk("errors", rx_buf[8 + 68 + i], snap_err[seq * 4 + i]);
            end
            if (en_frames % 5 == 4)
                $display("tb: encoder frame %0d t=%0d us: counts %0d %0d %0d %0d, errors %0d %0d %0d %0d",
                         seq, be32(48), $signed(be32(52)), $signed(be32(56)), $signed(be32(60)), $signed(be32(64)),
                         rx_buf[8 + 68], rx_buf[8 + 69], rx_buf[8 + 70], rx_buf[8 + 71]);
            en_frames = en_frames + 1;
        end
    end
endtask

always @(posedge clk) begin
    tx_en_d <= tx_en;
    if (tx_en) begin
        if (!tx_en_d) begin
            if (frames > 0 && gap < 48) begin
                $display("ERROR: inter-frame gap %0d cycles, minimum 48", gap);
                errors = errors + 1;
            end
            rx_len = 0;
            dibits = 0;
        end
        sh     = {txd, sh[7:2]};
        dibits = dibits + 1;
        if (dibits % 4 == 0 && rx_len < 2048) begin
            rx_buf[rx_len] = sh;
            rx_len = rx_len + 1;
        end
        gap = 0;
    end else begin
        gap = gap + 1;
        if (tx_en_d) check_frame;
    end
end

initial begin
    wait (en_frames == N_FRAMES);
    #1;
    // the encoders must have finished: compare the totals too
    chk("wheel 0 total", exp_count[0], 900);
    chk("wheel 1 total", exp_count[1], -250);
    chk("wheel 2 total", exp_count[2], 150);
    chk("wheel 3 total", exp_count[3], 98);
    chk("wheel 3 errors", exp_err[3], 2);
    chk("wheel 0 count changes", changes[0], 900);
    chk("wheel 1 count changes", changes[1], 250);
    chk("wheel 2 changes (spikes)", changes[2], 150);
    chk("wheel 3 count changes", changes[3], 98);
`ifndef SENSORS_TOP
    chk("LED1..4 (encoders moving)", led[4:1], 4'b1111);
    chk("LED7 (decode error seen)", led[7], 1'b1);
`endif
    if (errors == 0) $display("PASS: %0d encoder frames checked", en_frames);
    else             $display("FAIL: %0d errors", errors);
    $finish;
end

initial begin
    #20_000_000;
    $display("FAIL: timeout, %0d encoder frames", en_frames);
    $finish;
end

endmodule
