// Self-checking testbench for top/eth_tx.
// Samples the RMII TX pins on the rising edge of REF_CLK like the PHY does,
// rebuilds the bytes and checks for every frame:
//   - 7 x 0x55 preamble + 0xD5 SFD, whole number of bytes
//   - frame bytes == expected.hex (written from the MIF by udp_packet.py)
//   - FCS == CRC-32 of the frame (computed here, independent of crc32.v)
//   - inter-frame gap >= 96 bit times (48 REF_CLK cycles)
// Prints PASS or FAIL. Run it with run_sim.sh.

`timescale 1ns/1ps

module tb_eth_tx;

parameter N_FRAMES   = 3;
parameter TIMEOUT_NS = 1_000_000;

reg clk = 0;
always #10 clk = ~clk; // 50 MHz REF_CLK

wire [1:0] txd;
wire       tx_en;
wire       led;

top dut (
    .i_clk       ( clk   ),
    .o_eth_data  ( txd   ),
    .o_eth_tx_en ( tx_en ),
    .o_led_tx    ( led   )
);

//=============================================================================
reg [7:0] exp_mem [0:2047];
integer   exp_len;

reg [7:0] rx_buf [0:2047];
integer   rx_len;
integer   dibits;
reg [7:0] sh;
reg       tx_en_d = 0;

integer   frames = 0;
integer   errors = 0;
integer   gap    = 0;

integer   fd, i;
reg [7:0] b;

initial begin
    fd = $fopen("expected.hex", "r");
    if (fd == 0) begin
        $display("FAIL: cannot open expected.hex");
        $finish;
    end
    exp_len = 0;
    while ($fscanf(fd, "%h\n", b) == 1) begin
        exp_mem[exp_len] = b;
        exp_len = exp_len + 1;
    end
    $fclose(fd);
    $display("tb: expecting %0d-byte frames", exp_len);

    if ($test$plusargs("vcd")) begin
        $dumpfile("tb_eth_tx.vcd");
        $dumpvars(0, tb_eth_tx);
    end
end

initial begin
    #(TIMEOUT_NS);
    $display("FAIL: timeout after %0d ns, %0d of %0d frames seen", TIMEOUT_NS, frames, N_FRAMES);
    $finish;
end

// Ethernet CRC-32 (reflected, poly 0x04C11DB7), one byte at a time
function [31:0] crc32_byte(input [31:0] crc, input [7:0] d);
    integer k;
    begin
        crc32_byte = crc ^ d;
        for (k = 0; k < 8; k = k + 1)
            crc32_byte = crc32_byte[0] ? (crc32_byte >> 1) ^ 32'hEDB88320 : (crc32_byte >> 1);
    end
endfunction

task check_frame;
    integer    n, flen, bad;
    reg [31:0] crc, fcs;
    begin
        frames = frames + 1;
        flen   = rx_len - 8 - 4;

        if (dibits % 4 != 0) begin
            $display("ERROR frame %0d: %0d dibits is not a whole number of bytes", frames, dibits);
            errors = errors + 1;
        end

        bad = 0;
        for (n = 0; n < 8 && n < rx_len; n = n + 1)
            if (rx_buf[n] !== (n == 7 ? 8'hD5 : 8'h55)) bad = bad + 1;
        if (rx_len < 8 || bad) begin
            $display("ERROR frame %0d: bad preamble/SFD", frames);
            errors = errors + 1;
        end

        if (flen != exp_len) begin
            $display("ERROR frame %0d: %0d bytes between SFD and FCS, expected %0d", frames, flen, exp_len);
            errors = errors + 1;
        end
        if (flen + 4 < 64) begin
            $display("ERROR frame %0d: %0d bytes with FCS, Ethernet minimum is 64", frames, flen + 4);
            errors = errors + 1;
        end

        bad = 0;
        for (n = 0; n < flen && n < exp_len; n = n + 1)
            if (rx_buf[8 + n] !== exp_mem[n]) begin
                if (bad < 5)
                    $display("ERROR frame %0d: byte %0d = %02x, expected %02x", frames, n, rx_buf[8 + n], exp_mem[n]);
                bad = bad + 1;
            end
        if (bad) errors = errors + 1;

        if (flen > 0) begin
            crc = 32'hFFFFFFFF;
            for (n = 0; n < flen; n = n + 1)
                crc = crc32_byte(crc, rx_buf[8 + n]);
            crc = ~crc;
            fcs = {rx_buf[8 + flen + 3], rx_buf[8 + flen + 2], rx_buf[8 + flen + 1], rx_buf[8 + flen]};
            if (fcs !== crc) begin
                $display("ERROR frame %0d: FCS %08x, expected %08x", frames, fcs, crc);
                errors = errors + 1;
            end else
                $display("tb: frame %0d at %0d ns: %0d bytes + FCS %08x OK", frames, $time, flen, fcs);
        end

        if (frames == N_FRAMES) begin
            if (errors == 0) $display("PASS: %0d frames checked", frames);
            else             $display("FAIL: %0d errors in %0d frames", errors, frames);
            $finish;
        end
    end
endtask

always @(posedge clk) begin
    tx_en_d <= tx_en;
    if (tx_en) begin
        if (!tx_en_d) begin
            if (frames > 0 && gap < 48) begin
                $display("ERROR: inter-frame gap %0d cycles, minimum is 48", gap);
                errors = errors + 1;
            end
            rx_len = 0;
            dibits = 0;
        end
        sh     = {txd, sh[7:2]}; // RMII sends the low dibit of each byte first
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
