// Behavioural stand-in for ram_ip_paket_tx (altsyncram) for simulators
// without the Altera libraries, e.g. Icarus Verilog.
// Same read timing as the IP: address/rden registered and q registered
// (address_reg_b = outdata_reg_b = CLOCK1), so q lags rdaddress by 2 clocks.
// Contents come from expected.hex, which run_sim.sh makes from the MIF.

module ram_ip_paket_tx (
    input  wire [7:0]  data,
    input  wire [10:0] rdaddress,
    input  wire        rdclock,
    input  wire        rden,
    input  wire [10:0] wraddress,
    input  wire        wrclock,
    input  wire        wren,
    output wire [7:0]  q
);

reg [7:0] mem [0:2047];
reg [7:0] q_ram = 0;
reg [7:0] q_out = 0;
integer   i, fd;
reg [7:0] b;

initial begin
    for (i = 0; i < 2048; i = i + 1) mem[i] = 0;
    fd = $fopen("expected.hex", "r");
    i  = 0;
    if (fd != 0) begin
        while ($fscanf(fd, "%h\n", b) == 1) begin
            mem[i] = b;
            i = i + 1;
        end
        $fclose(fd);
    end
end

always @(posedge wrclock)
    if (wren) mem[wraddress] <= data;

always @(posedge rdclock) begin
    if (rden) q_ram <= mem[rdaddress];
    q_out <= q_ram;
end

assign q = q_out;

endmodule
