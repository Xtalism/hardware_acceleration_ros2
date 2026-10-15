# 50 MHz RMII REF_CLK from the PHY module (nINT/RETCLK -> i_clk)
create_clock -name rmii_clk -period 20.000 [get_ports i_clk]
derive_clock_uncertainty

# The PHY samples TXD[1:0]/TX_EN on the rising edge of REF_CLK (RMII: Tsu 4 ns, Th 2 ns)
set_output_delay -clock rmii_clk -max  4.0 [get_ports {o_eth_data[*] o_eth_tx_en}]
set_output_delay -clock rmii_clk -min -2.0 [get_ports {o_eth_data[*] o_eth_tx_en}]

# ADXL345 SPI at 2 MHz (timing set by the state machine) and LEDs
set_false_path -to   [get_ports {o_g_cs_n o_g_sclk io_g_sdio o_led[*]}]
set_false_path -from [get_ports io_g_sdio]
