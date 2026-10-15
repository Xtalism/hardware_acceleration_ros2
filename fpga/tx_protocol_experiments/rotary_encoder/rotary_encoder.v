module rotary_encoder(
    clk, 
    encoder_a, 
    encoder_b,  
    switch_in, 
    switch_out, 
    direction_led, 
    count
);

input clk, encoder_a, encoder_b, switch_in;
output switch_out, direction_led;
output [7:0] count;

reg [2:0] encoder_a_delayed, encoder_b_delayed;

always @(posedge clk)
begin
    encoder_a_delayed <= {encoder_a_delayed[1:0], encoder_a};
end

always @(posedge clk)
begin
    encoder_b_delayed <= {encoder_b_delayed[1:0], encoder_b};
end

wire count_enable = encoder_a_delayed[1] ^ encoder_a_delayed[2] ^ encoder_b_delayed[1] ^ encoder_b_delayed[2];
wire count_direction = encoder_a_delayed[1] ^ encoder_b_delayed[2];

assign switch_out = switch_in;
assign direction_led = count_direction;

reg[7:0] count;

always @(posedge clk)
begin
    if(count_enable)
    begin
        if(count_direction)
            begin
                count <= count + 1;
            end
        else
            begin
                count <= count - 1;
            end
    end
end

endmodule
