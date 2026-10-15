module uart_rx_ld19 (
    input clk,            // 50MHz
    input rx_pin,         // PIN_B8
    output reg [7:0] data_out,
    output reg data_valid
);
    parameter CLKS_PER_BIT = 217; // 50MHz / 230400
    
    localparam IDLE         = 3'b000;
    localparam RX_START_BIT = 3'b001;
    localparam RX_DATA_BITS = 3'b010;
    localparam RX_STOP_BIT  = 3'b011;
    localparam CLEANUP      = 3'b100;
    
    reg [2:0] state = IDLE;
    reg [15:0] clk_count = 0;
    reg [2:0] bit_index = 0;
    reg [7:0] shift_reg = 0;
    
    // Double-register the RX pin to prevent metastability
    reg rx_data_r;
    reg rx_data;
    always @(posedge clk) begin
        rx_data_r <= rx_pin;
        rx_data   <= rx_data_r;
    end

    always @(posedge clk) begin
        case (state)
            IDLE: begin
                data_valid <= 0;
                clk_count <= 0;
                bit_index <= 0;
                if (rx_data == 1'b0) // Detected Start Bit
                    state <= RX_START_BIT;
                else
                    state <= IDLE;
            end
            
            RX_START_BIT: begin
                if (clk_count == (CLKS_PER_BIT / 2)) begin
                    if (rx_data == 1'b0) begin
                        clk_count <= 0;  // reset counter, found center of start bit
                        state <= RX_DATA_BITS;
                    end else begin
                        state <= IDLE;
                    end
                end else begin
                    clk_count <= clk_count + 1;
                    state <= RX_START_BIT;
                end
            end
            
            RX_DATA_BITS: begin
                if (clk_count == CLKS_PER_BIT - 1) begin
                    clk_count <= 0;
                    shift_reg[bit_index] <= rx_data;
                    
                    if (bit_index < 7) begin
                        bit_index <= bit_index + 1;
                        state <= RX_DATA_BITS;
                    end else begin
                        bit_index <= 0;
                        state <= RX_STOP_BIT;
                    end
                end else begin
                    clk_count <= clk_count + 1;
                    state <= RX_DATA_BITS;
                end
            end
            
            RX_STOP_BIT: begin
                // Wait CLKS_PER_BIT-1 clock cycles for Stop bit to finish
                if (clk_count == CLKS_PER_BIT - 1) begin
                    data_out <= shift_reg;
                    data_valid <= 1'b1;
                    clk_count <= 0;
                    state <= CLEANUP;
                end else begin
                    clk_count <= clk_count + 1;
                    state <= RX_STOP_BIT;
                end
            end
            
            CLEANUP: begin
                state <= IDLE;
                data_valid <= 1'b0;
            end
            
            default: state <= IDLE;
        endcase
    end
endmodule