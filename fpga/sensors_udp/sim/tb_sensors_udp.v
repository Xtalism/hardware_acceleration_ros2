// Self-checking testbench for sensors_udp_top (encoder decoding is tested in
// fpga/encoder_udp/sim/tb_encoder_udp.v; here the encoders are idle): a behavioural ADXL345
// (../../gsensor_udp/sim/adxl345_model.v) and a behavioural
// LD19 (UART 230400: junk, the manual's example packet, generated packets,
// one with a bad CRC) run at the same time. The ADXL345 runs much faster than
// the real one so the two streams often compete for the Ethernet link.
// Every frame is decoded like the PHY does and checked by its UDP port:
//   both:  preamble/SFD, FCS, inter-frame gap, header == expected_*_hdr.hex
//   1024:  "GS", sequence 0,1,2..., X/Y/Z of model sample seq+1, DEVID, flags,
//          FPGA time 20..50 us after the model produced the sample
//   1025:  "LD", sequence 0,1,2..., bad-CRC count, the 47 LD19 bytes,
//          FPGA time ~9.5 bit times after the packet's first start bit
// The two time checks use the same simulation clock, which shows both
// sensors are stamped by one counter. Also checks that the bad LD19 packet
// produced no frame, and that each sensor had to wait for the other's frame
// at least once (so the arbiter was exercised). Prints PASS or FAIL.

`timescale 1ns/1ps

module tb_sensors_udp;

parameter DR_PERIOD_NS = 50_000;            // ADXL345 sample every 50 us here
parameter TIMEOUT_NS   = 30_000_000;

localparam N_PKT     = 10;                  // LD19 packets sent
localparam BAD_PKT   = 3;                   // this one has a bad CRC
localparam LD_FRAMES = N_PKT - 1;

reg clk = 0;
always #10 clk = ~clk;                      // 50 MHz REF_CLK

reg        lidar = 1'b1;
wire [1:0] txd;
wire       tx_en;
wire       cs_n, sclk, sdio;
wire [7:0] led;

pullup (sdio);

sensors_udp_top #( .STARTUP_CYCLES(1000), .ENC_PERIOD_US(300) ) dut
(
    .i_clk       ( clk   ),
    .o_eth_data  ( txd   ),
    .o_eth_tx_en ( tx_en ),
    .o_g_cs_n    ( cs_n  ),
    .o_g_sclk    ( sclk  ),
    .io_g_sdio   ( sdio  ),
    .i_lidar_rx  ( lidar ),
    .i_enc_a     ( 4'b0000 ),           // encoders idle here: see tb_encoder_udp.v
    .i_enc_b     ( 4'b0000 ),
    .o_led       ( led   )
);

adxl345_model #( .DR_PERIOD_NS(DR_PERIOD_NS) ) adxl ( .cs_n(cs_n), .sclk(sclk), .sdio(sdio) );

// when the ADXL345 model produced each sample (us)
real gs_t [0:4095];
always @(adxl.sample) if (adxl.sample < 4096) gs_t[adxl.sample] = $realtime / 1000.0;

//=============================================================================
// LD19 model
reg  [7:0] pk [0:N_PKT*47-1];
real       ld_start [0:N_PKT-1];
real       bit_ns;
reg  [7:0] example [0:46];
integer    j;
reg        all_sent = 0;

function [7:0] crc8(input [7:0] c, input [7:0] d);
    integer k;
    begin
        crc8 = c ^ d;
        for (k = 0; k < 8; k = k + 1)
            crc8 = crc8[7] ? {crc8[6:0], 1'b0} ^ 8'h4D : {crc8[6:0], 1'b0};
    end
endfunction

task set16(input integer idx, input integer off, input [15:0] v);
    begin
        pk[idx * 47 + off]     = v[7:0];
        pk[idx * 47 + off + 1] = v[15:8];
    end
endtask

task make_packet(input integer idx);
    integer i;
    reg [7:0] c;
    begin
        pk[idx * 47]     = 8'h54;
        pk[idx * 47 + 1] = 8'h2C;
        set16(idx, 2, 16'd3600);
        set16(idx, 4, (idx * 1200) % 36000);
        for (i = 0; i < 12; i = i + 1) begin
            set16(idx, 6 + 3 * i, 100 + 37 * idx + i);
            pk[idx * 47 + 8 + 3 * i] = 8'd200 + i;
        end
        set16(idx, 42, (idx * 1200 + 1100) % 36000);
        set16(idx, 44, 3 * idx);
        c = 0;
        for (i = 0; i < 46; i = i + 1) c = crc8(c, pk[idx * 47 + i]);
        pk[idx * 47 + 46] = c;
    end
endtask

task send_byte(input [7:0] b);
    integer i;
    begin
        lidar = 1'b0;
        #(bit_ns);
        for (i = 0; i < 8; i = i + 1) begin
            lidar = b[i];
            #(bit_ns);
        end
        lidar = 1'b1;
        #(bit_ns);
    end
endtask

task send_packet(input integer idx);
    integer i;
    begin
        ld_start[idx] = $realtime / 1000.0;
        for (i = 0; i < 47; i = i + 1) send_byte(pk[idx * 47 + i]);
        #(bit_ns * (3 + 17 * idx));         // varying idle so packets land at different moments
    end
endtask

initial begin
    bit_ns = 1.0e9 / 230400.0;
    $readmemh("ld19_example.hex", example);
    for (j = 0; j < 47; j = j + 1) pk[j] = example[j];
    for (j = 1; j < N_PKT; j = j + 1) make_packet(j);
    pk[BAD_PKT * 47 + 46] = pk[BAD_PKT * 47 + 46] ^ 8'h01;

    #300_000;                               // let the ADXL345 start first
    send_byte(8'h12);
    send_byte(8'h54);
    send_byte(8'h00);
    for (j = 0; j < N_PKT; j = j + 1) send_packet(j);
    all_sent = 1;
end

//=============================================================================
// Frame checker
reg [7:0]  gs_hdr [0:59];
reg [7:0]  ld_hdr [0:100];
reg [7:0]  en_hdr [0:71];
integer    en_frames = 0;
reg [31:0] en_t_prev = 0;
reg [7:0]  rx_buf [0:2047];
integer    rx_len, dibits, frames = 0, gs_frames = 0, ld_frames = 0, errors = 0, gap = 0;
integer    ld_waited = 0, gs_waited = 0;  // a packet waited for the other sensor's frame
reg        ld_wait_q = 0, gs_wait_q = 0;
reg [7:0]  sh;
reg        tx_en_d = 0;


initial begin
    $readmemh("expected_gs_hdr.hex", gs_hdr);   // whole 60-byte frame; bytes 0..41 are the header
    $readmemh("expected_ld_hdr.hex", ld_hdr);   // whole 101-byte frame
    $readmemh("expected_en_hdr.hex", en_hdr);   // whole 72-byte frame
    if ($test$plusargs("vcd")) begin
        $dumpfile("tb_sensors_udp.vcd");
        $dumpvars(0, tb_sensors_udp);
    end
end

initial begin
    #(TIMEOUT_NS);
    $display("FAIL: timeout");
    $finish;
end

// arbiter coverage: one sensor's packet waiting while the other's frame is sent
wire ld_wait = dut.ld_pending && dut.tx_st != 0 && dut.sel != 2'd1;
wire gs_wait = dut.gs_pending && dut.tx_st != 0 && dut.sel == 2'd1;
wire en_wait = dut.en_pending && dut.tx_st != 0 && dut.sel != 2'd2;
integer en_waited = 0;
reg     en_wait_q = 0;
always @(posedge clk) begin
    en_wait_q <= en_wait;
    if (en_wait && !en_wait_q) en_waited = en_waited + 1;
end
always @(posedge clk) begin
    ld_wait_q <= ld_wait;
    gs_wait_q <= gs_wait;
    if (ld_wait && !ld_wait_q) ld_waited = ld_waited + 1;
    if (gs_wait && !gs_wait_q) gs_waited = gs_waited + 1;
end

function [31:0] crc32_byte(input [31:0] crc, input [7:0] d);
    integer k;
    begin
        crc32_byte = crc ^ d;
        for (k = 0; k < 8; k = k + 1)
            crc32_byte = crc32_byte[0] ? (crc32_byte >> 1) ^ 32'hEDB88320 : (crc32_byte >> 1);
    end
endfunction

function [31:0] be32(input integer off);
    be32 = {rx_buf[8 + off], rx_buf[9 + off], rx_buf[10 + off], rx_buf[11 + off]};
endfunction

function [15:0] be16(input integer off);
    be16 = {rx_buf[8 + off], rx_buf[9 + off]};
endfunction

task chk(input [255:0] what, input [31:0] got, input [31:0] want);
    if (got !== want) begin
        $display("ERROR frame %0d: %0s = %0h, expected %0h", frames, what, got, want);
        errors = errors + 1;
    end
endtask

task check_gs;
    integer i, bad, s;
    real    dt;
    begin
        bad = 0;
        for (i = 0; i < 42; i = i + 1) if (rx_buf[8 + i] !== gs_hdr[i]) bad = bad + 1;
        chk("G-sensor header bytes wrong", bad, 0);
        chk("frame length", rx_len - 12, 60);
        s = gs_frames + 1;                                  // model sample number
        chk("magic GS",   be16(42), 16'h4753);
        chk("G-sensor sequence", be32(44), gs_frames);
        chk("X", be16(52), (7 * s - 100) & 16'hFFFF);
        chk("Y", be16(54), (-3 * s) & 16'hFFFF);
        chk("Z", be16(56), 256 + s);
        chk("DEVID", rx_buf[8 + 58], 8'hE5);
        chk("flags", rx_buf[8 + 59], 8'h00);
        dt = be32(48) - gs_t[s];
        if (dt < 20.0 || dt > 50.0) begin
            $display("ERROR frame %0d: G-sensor time %0d us is %0.1f us after the sample, expected 20..50", frames, be32(48), dt);
            errors = errors + 1;
        end
        gs_frames = gs_frames + 1;
    end
endtask

task check_ld;
    integer i, bad, p;
    real    t_exp;
    begin
        bad = 0;
        for (i = 0; i < 42; i = i + 1) if (rx_buf[8 + i] !== ld_hdr[i]) bad = bad + 1;
        chk("LD19 header bytes wrong", bad, 0);
        chk("frame length", rx_len - 12, 101);
        p = (ld_frames < BAD_PKT) ? ld_frames : ld_frames + 1;
        chk("magic LD", be16(42), 16'h4C44);
        chk("LD19 sequence", be32(44), ld_frames);
        chk("bad CRC count", be16(52), p > BAD_PKT ? 1 : 0);
        bad = 0;
        for (i = 0; i < 47; i = i + 1) if (rx_buf[8 + 54 + i] !== pk[p * 47 + i]) bad = bad + 1;
        chk("LD19 bytes wrong", bad, 0);
        t_exp = ld_start[p] + 9.5 * bit_ns / 1000.0;
        if (be32(48) < t_exp - 3.0 || be32(48) > t_exp + 3.0) begin
            $display("ERROR frame %0d: LD19 time %0d us, expected about %0.1f", frames, be32(48), t_exp);
            errors = errors + 1;
        end
        $display("tb: LD19 frame %0d (packet %0d) at %0d us, FPGA time %0d us, G-sensor frames so far %0d",
                 ld_frames, p, $time / 1000, be32(48), gs_frames);
        ld_frames = ld_frames + 1;
    end
endtask

task check_en;
    integer i, bad;
    begin
        bad = 0;
        for (i = 0; i < 42; i = i + 1) if (rx_buf[8 + i] !== en_hdr[i]) bad = bad + 1;
        chk("encoder header bytes wrong", bad, 0);
        chk("frame length", rx_len - 12, 72);
        chk("magic EN", be16(42), 16'h454E);
        chk("encoder sequence", be32(44), en_frames);
        if (en_frames > 0) chk("encoder time step", be32(48) - en_t_prev, 300);
        en_t_prev = be32(48);
        for (i = 0; i < 4; i = i + 1) chk("encoder count", be32(52 + 4 * i), 0);
        chk("encoder errors", be32(68), 0);
        en_frames = en_frames + 1;
    end
endtask

task check_frame;
    integer    i, flen, bad;
    reg [31:0] crc, fcs;
    begin
        frames = frames + 1;
        flen   = rx_len - 12;
        if (dibits % 4 != 0) begin
            $display("ERROR frame %0d: %0d dibits is not whole bytes", frames, dibits);
            errors = errors + 1;
        end
        bad = 0;
        for (i = 0; i < 8; i = i + 1)
            if (rx_buf[i] !== (i == 7 ? 8'hD5 : 8'h55)) bad = 1;
        chk("bad preamble/SFD", bad, 0);
        crc = 32'hFFFFFFFF;
        for (i = 0; i < flen; i = i + 1) crc = crc32_byte(crc, rx_buf[8 + i]);
        fcs = {rx_buf[8 + flen + 3], rx_buf[8 + flen + 2], rx_buf[8 + flen + 1], rx_buf[8 + flen]};
        chk("FCS", fcs, ~crc);

        case (be16(36))                                     // UDP destination port
        16'd1024: check_gs;
        16'd1025: check_ld;
        16'd1026: check_en;
        default: begin
            $display("ERROR frame %0d: unexpected UDP port %0d", frames, be16(36));
            errors = errors + 1;
        end
        endcase
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
    wait (all_sent);
    #200_000;
    chk("LD19 frames", ld_frames, LD_FRAMES);
    if (gs_frames < 100) begin
        $display("ERROR: only %0d G-sensor frames", gs_frames);
        errors = errors + 1;
    end
    if (en_frames < 30) begin
        $display("ERROR: only %0d encoder frames", en_frames);
        errors = errors + 1;
    end
    if (ld_waited == 0 || gs_waited == 0 || en_waited == 0) begin
        $display("ERROR: arbitration not exercised (LD19 waited %0d, G-sensor %0d, encoders %0d times)", ld_waited, gs_waited, en_waited);
        errors = errors + 1;
    end
    chk("LED1 (DEVID ok)", led[1], 1'b1);
    chk("LED3 (LD19 bytes)", led[3], 1'b1);
    chk("LED7 (bad CRC seen)", led[7], 1'b1);
    errors = errors + adxl.errors;
    $display("tb: %0d G-sensor, %0d LD19, %0d encoder frames; waited for another frame: LD19 %0d, G-sensor %0d (behind LD19), encoders %0d times",
             gs_frames, ld_frames, en_frames, ld_waited, gs_waited, en_waited);
    if (errors == 0) $display("PASS: %0d frames checked", frames);
    else             $display("FAIL: %0d errors", errors);
    $finish;
end

endmodule
