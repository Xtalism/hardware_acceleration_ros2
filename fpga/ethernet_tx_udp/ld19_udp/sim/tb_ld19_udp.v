// Self-checking testbench for ld19_udp_top.
// A behavioural LD19 sends, over UART at 230400 baud (+ BAUD_ERR_PPM):
//   junk bytes (incl. a lone 0x54), the example packet from the LD19 manual
//   (section 3.3), generated packets, and one packet with a corrupted CRC.
// The RMII TX pins are decoded like the PHY does and every frame is checked:
//   preamble/SFD, FCS, inter-frame gap, Ethernet/IP/UDP header against
//   expected_hdr.hex (from udp_packet.py), "LD", sequence 0,1,2..., the bad-CRC
//   count, the 47 LD19 bytes unchanged, and the FPGA timestamp of each packet.
// The corrupted packet must not produce a frame. Prints PASS or FAIL.

`timescale 1ns/1ps

module tb_ld19_udp;

parameter BAUD_ERR_PPM = 0;
parameter TIMEOUT_NS   = 20_000_000;

localparam N_PKT    = 5;                    // packets the LD19 sends
localparam N_FRAMES = 4;                    // packet 3 has a bad CRC
localparam BAD_PKT  = 3;

reg clk = 0;
always #10 clk = ~clk;                      // 50 MHz REF_CLK

reg        lidar = 1'b1;                    // UART idle high
wire [1:0] txd;
wire       tx_en;
wire [7:0] led;

ld19_udp_top dut
(
    .i_clk       ( clk   ),
    .o_eth_data  ( txd   ),
    .o_eth_tx_en ( tx_en ),
    .i_lidar_rx  ( lidar ),
    .o_led       ( led   )
);

//=============================================================================
// LD19 model
reg  [7:0] pk [0:N_PKT*47-1];
real       start_us [0:N_PKT-1];            // when each packet's first start bit began
real       bit_ns;

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
        pk[idx * 47 + off]     = v[7:0];    // LD19 fields are little-endian
        pk[idx * 47 + off + 1] = v[15:8];
    end
endtask

task make_packet(input integer idx);
    integer i;
    reg [7:0] c;
    begin
        pk[idx * 47]     = 8'h54;
        pk[idx * 47 + 1] = 8'h2C;
        set16(idx, 2, 16'd3600);                        // 3600 deg/s = 10 Hz
        set16(idx, 4, (idx * 1200) % 36000);            // start angle, 0.01 deg
        for (i = 0; i < 12; i = i + 1) begin
            set16(idx, 6 + 3 * i, 100 + 37 * idx + i);  // distance, mm
            pk[idx * 47 + 8 + 3 * i] = 8'd200 + i;      // intensity
        end
        set16(idx, 42, (idx * 1200 + 1100) % 36000);    // end angle
        set16(idx, 44, 3 * idx);                        // LD19 timestamp, ms
        c = 0;
        for (i = 0; i < 46; i = i + 1) c = crc8(c, pk[idx * 47 + i]);
        pk[idx * 47 + 46] = c;
    end
endtask

task send_byte(input [7:0] b);
    integer i;
    begin
        lidar = 1'b0;                       // start bit
        #(bit_ns);
        for (i = 0; i < 8; i = i + 1) begin
            lidar = b[i];
            #(bit_ns);
        end
        lidar = 1'b1;                       // stop bit
        #(bit_ns);
    end
endtask

task send_packet(input integer idx);
    integer i;
    begin
        start_us[idx] = $realtime / 1000.0;
        for (i = 0; i < 47; i = i + 1) send_byte(pk[idx * 47 + i]);
        #(bit_ns * 13);                     // short idle between packets
    end
endtask

reg [7:0] example [0:46];
integer   j;
reg       all_sent = 0;

initial begin
    bit_ns = 1.0e9 / (230400.0 * (1.0 + BAUD_ERR_PPM / 1.0e6));

    // packet 0: the example from the LD19 manual, section 3.3
    $readmemh("ld19_example.hex", example);
    for (j = 0; j < 47; j = j + 1) pk[j] = example[j];
    for (j = 1; j < N_PKT; j = j + 1) make_packet(j);
    pk[BAD_PKT * 47 + 46] = pk[BAD_PKT * 47 + 46] ^ 8'h01;   // corrupt the CRC

    #50_000;
    send_byte(8'h12);                       // junk, including a lone header byte
    send_byte(8'h54);
    send_byte(8'h00);
    #20_000;
    for (j = 0; j < N_PKT; j = j + 1) send_packet(j);
    all_sent = 1;
end

//=============================================================================
// Frame checker
reg [7:0]  hdr [0:59];
reg [7:0]  rx_buf [0:2047];
integer    rx_len, dibits, frames = 0, errors = 0, gap = 0;
reg [7:0]  sh;
reg        tx_en_d = 0;
integer    fd, n;
reg [7:0]  b;

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
        $dumpfile("tb_ld19_udp.vcd");
        $dumpvars(0, tb_ld19_udp);
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
    integer    i, flen, bad, p;
    reg [31:0] crc, fcs;
    real       t_exp;
    begin
        frames = frames + 1;
        flen   = rx_len - 12;
        p      = (frames - 1 < BAD_PKT) ? frames - 1 : frames;   // LD19 packet in this frame

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
        chk("frame length", flen, 101);

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

        chk("magic",    be16(42), 16'h4C44);
        chk("sequence", be32(44), frames - 1);
        chk("bad CRC count", be16(52), p > BAD_PKT ? 1 : 0);

        bad = 0;
        for (i = 0; i < 47; i = i + 1)
            if (rx_buf[8 + 54 + i] !== pk[p * 47 + i]) begin
                if (bad < 3) $display("ERROR frame %0d: LD19 byte %0d = %02x, expected %02x", frames, i, rx_buf[8 + 54 + i], pk[p * 47 + i]);
                bad = bad + 1;
            end
        if (bad) errors = errors + 1;

        // first byte complete ~9.5 bit times after its start bit
        t_exp = start_us[p] + 9.5 * bit_ns / 1000.0;
        if (be32(48) < t_exp - 3.0 || be32(48) > t_exp + 3.0) begin
            $display("ERROR frame %0d: FPGA time %0d us, expected about %0.1f", frames, be32(48), t_exp);
            errors = errors + 1;
        end

        $display("tb: frame %0d at %0d us: seq %0d, LD19 packet %0d, start angle %0d, first distance %0d mm, bad %0d, t=%0d us",
                 frames, $time / 1000, be32(44), p, {rx_buf[8 + 59], rx_buf[8 + 58]},
                 {rx_buf[8 + 61], rx_buf[8 + 60]}, be16(52), be32(48));
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

// after the last packet: exactly N_FRAMES frames, LD19 activity LED on
initial begin
    wait (all_sent);
    #200_000;
    chk("frames sent", frames, N_FRAMES);
    chk("LED1 (bytes arriving)", led[1], 1'b1);
    chk("LED7 (bad CRC seen)", led[7], 1'b1);
    if (errors == 0) $display("PASS: %0d frames checked (baud error %0d ppm)", frames, BAUD_ERR_PPM);
    else             $display("FAIL: %0d errors (baud error %0d ppm)", errors, BAUD_ERR_PPM);
    $finish;
end

endmodule
