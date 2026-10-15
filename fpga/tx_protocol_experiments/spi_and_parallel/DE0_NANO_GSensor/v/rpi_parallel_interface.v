module rpi_parallel_interface (
    input             iRSTN,
    input             iCLK,
    input      [15:0] iDATA_X,
    input      [15:0] iDATA_Y,
    input      [15:0] iDATA_Z,
    input             iRP_CLK,
    input             iRP_CS,
    output reg [7:0]  oRP_DATA
);

reg [47:0] data_buffer;  // 3 axes * 16 bits = 48 bits
reg [2:0]  byte_counter;
reg        rp_clk_d1, rp_clk_d2;
reg        rp_cs_d1, rp_cs_d2;

// Synchronize RP_CLK and RP_CS to FPGA clock domain
always @(posedge iCLK or negedge iRSTN) begin
    if (!iRSTN) begin
        rp_clk_d1 <= 1'b0;
        rp_clk_d2 <= 1'b0;
        rp_cs_d1  <= 1'b0;
        rp_cs_d2  <= 1'b0;
    end else begin
        rp_clk_d1 <= iRP_CLK;
        rp_clk_d2 <= rp_clk_d1;
        rp_cs_d1  <= iRP_CS;
        rp_cs_d2  <= rp_cs_d1;
    end
end

// Detect rising edge of RP_CLK
wire rp_clk_rising = rp_clk_d1 && !rp_clk_d2;

// Update data buffer when CS goes high (start of transaction)
wire cs_rising = rp_cs_d1 && !rp_cs_d2;

always @(posedge iCLK or negedge iRSTN) begin
    if (!iRSTN) begin
        data_buffer <= 48'h000000000000;
    end else if (cs_rising || !rp_cs_d2) begin
        // Latch all three axes: X, Y, Z (each 16 bits)
        data_buffer <= {iDATA_Z, iDATA_Y, iDATA_X};
    end
end

// Handle byte transmission
always @(posedge iCLK or negedge iRSTN) begin
    if (!iRSTN) begin
        byte_counter <= 3'd0;
        oRP_DATA     <= 8'h00;
    end else if (!rp_cs_d2) begin
        // CS inactive - reset
        byte_counter <= 3'd0;
        oRP_DATA     <= 8'h00;
    end else begin
        // CS active - send data on clock rising edge
        if (rp_clk_rising) begin
            byte_counter <= byte_counter + 3'd1;
        end
        
        // Output the correct byte based on counter
        case (byte_counter)
            3'd0: oRP_DATA <= data_buffer[7:0];    // X low byte
            3'd1: oRP_DATA <= data_buffer[15:8];   // X high byte
            3'd2: oRP_DATA <= data_buffer[23:16];  // Y low byte
            3'd3: oRP_DATA <= data_buffer[31:24];  // Y high byte
            3'd4: oRP_DATA <= data_buffer[39:32];  // Z low byte
            3'd5: oRP_DATA <= data_buffer[47:40];  // Z high byte
            default: oRP_DATA <= 8'h00;
        endcase
    end
end

endmodule