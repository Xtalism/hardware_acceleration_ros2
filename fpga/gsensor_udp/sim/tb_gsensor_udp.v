// Self-checking testbench for gsensor_udp_top with a behavioural ADXL345.
// Decodes the RMII TX pins like the PHY and checks every frame:
//   - preamble/SFD, whole bytes, FCS, inter-frame gap >= 48 REF_CLK cycles
//   - Ethernet/IP/UDP header == expected_hdr.hex (from udp_packet.py), which
//     also proves the IP checksum the FPGA computes
//   - payload: "GS", sequence 0,1,2..., X/Y/Z of model sample seq+1,
//     DEVID 0xE5, no dropped samples, timestamps one sample period apart
// and checks the ADXL345 registers the FPGA configured. Prints PASS or FAIL.

`timescale 1ns/1ps

module tb_gsensor_udp;

parameter N_FRAMES     = 6;
parameter DR_PERIOD_NS = 100_000;          // 10 kHz in simulation to keep it short
parameter TIMEOUT_NS   = 2_000_000;

reg clk = 0;
always #10 clk = ~clk;                     // 50 MHz REF_CLK

wire [1:0] txd;
wire       tx_en;
wire       cs_n, sclk, sdio;
wire [7:0] led;

pullup (sdio);                             // board pull-up on I2C_SDAT

gsensor_udp_top
#(
    .STARTUP_CYCLES ( 1000 )
)
dut
(
    .i_clk       ( clk   ),
    .o_eth_data  ( txd   ),
    .o_eth_tx_en ( tx_en ),
    .o_g_cs_n    ( cs_n  ),
    .o_g_sclk    ( sclk  ),
    .io_g_sdio   ( sdio  ),
    .o_led       ( led   )
);

adxl345_model #( .DR_PERIOD_NS(DR_PERIOD_NS) ) adxl ( .cs_n(cs_n), .sclk(sclk), .sdio(sdio) );

//=============================================================================
reg [7:0] hdr [0:59];
reg [7:0] rx_buf [0:2047];
integer   rx_len, dibits, frames = 0, errors = 0, gap = 0;
reg [7:0] sh;
reg       tx_en_d = 0;
integer   fd, n;
reg [7:0] b;
reg [31:0] prev_t;

initial begin
    fd = $fopen("expected_hdr.hex", "r");
    if (fd == 0) begin
        $display("FAIL: cannot open expected_hdr.hex");
        $finish;
    end
    n = 0;
    while ($fscanf(fd, "%h\n", b) == 1) begin
        hdr[n] = b;
        n = n + 1;
    end
    $fclose(fd);
    if ($test$plusargs("vcd")) begin
        $dumpfile("tb_gsensor_udp.vcd");
        $dumpvars(0, tb_gsensor_udp);
    end
end

initial begin
    #(TIMEOUT_NS);
    $display("FAIL: timeout, %0d of %0d frames seen", frames, N_FRAMES);
    $finish;
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

task check_frame;
    integer    i, flen, bad, s;
    reg [31:0] crc, fcs, t;
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
        if (bad) begin
            $display("ERROR frame %0d: bad preamble/SFD", frames);
            errors = errors + 1;
        end
        chk("frame length", flen, 60);

        crc = 32'hFFFFFFFF;
        for (i = 0; i < flen; i = i + 1) crc = crc32_byte(crc, rx_buf[8 + i]);
        fcs = {rx_buf[8 + flen + 3], rx_buf[8 + flen + 2], rx_buf[8 + flen + 1], rx_buf[8 + flen]};
        chk("FCS", fcs, ~crc);

        bad = 0;
        for (i = 0; i < 42; i = i + 1)
            if (rx_buf[8 + i] !== hdr[i]) begin
                if (bad < 3) $display("ERROR frame %0d: header byte %0d = %02x, expected %02x", frames, i, rx_buf[8 + i], hdr[i]);
                bad = bad + 1;
            end
        if (bad) errors = errors + 1;

        // payload starts at frame byte 42
        s = frames;                                     // model sample number
        chk("magic",    be16(42), 16'h4753);
        chk("sequence", be32(44), frames - 1);
        chk("X",        be16(52), (7 * s - 100) & 16'hFFFF);
        chk("Y",        be16(54), (-3 * s) & 16'hFFFF);
        chk("Z",        be16(56), 256 + s);
        chk("DEVID",    rx_buf[8 + 58], 8'hE5);
        chk("flags",    rx_buf[8 + 59], 8'h00);

        t = be32(48);
        if (frames > 1 && (t - prev_t < DR_PERIOD_NS / 1000 - 15 || t - prev_t > DR_PERIOD_NS / 1000 + 15)) begin
            $display("ERROR frame %0d: timestamp step %0d us, expected about %0d", frames, t - prev_t, DR_PERIOD_NS / 1000);
            errors = errors + 1;
        end
        prev_t = t;

        $display("tb: frame %0d at %0d ns: seq %0d t=%0d us X=%0d Y=%0d Z=%0d FCS ok=%0d",
                 frames, $time, be32(44), t, $signed(be16(52)), $signed(be16(54)), $signed(be16(56)), fcs == ~crc);

        if (frames == N_FRAMES) begin
            chk("ADXL345 DATA_FORMAT", adxl.regs[6'h31], 8'h4B);
            chk("ADXL345 BW_RATE",     adxl.regs[6'h2C], 8'h0C);
            chk("ADXL345 POWER_CTL",   adxl.regs[6'h2D], 8'h08);
            chk("LED1 (DEVID ok)",     led[1], 1'b1);
            errors = errors + adxl.errors;
            if (errors == 0) $display("PASS: %0d frames checked", frames);
            else             $display("FAIL: %0d errors", errors);
            $finish;
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

endmodule
