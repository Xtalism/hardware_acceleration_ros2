#!/usr/bin/env bash
# Run the self-checking testbenches of sensors_udp_top with Icarus Verilog:
#   tb_sensors_udp.v  ADXL345 + LD19 + idle encoders, all three packet types
#   ../../encoder_udp/sim/tb_encoder_udp.v
#                     the four wheel-encoder decoders with real A/B waveforms,
#                     run here against the combined design (-DSENSORS_TOP)
#
#   ./run_sim.sh          run both, prints PASS or FAIL for each
#   ./run_sim.sh +vcd     also dump build/*.vcd (open with gtkwave)
#
# Exit status 0 when both pass.
set -euo pipefail

SIM_DIR=$(cd "$(dirname "$0")" && pwd)
PRJ=$(dirname "$SIM_DIR")
FPGA=$(dirname "$PRJ")
REPO=$(dirname "$FPGA")
GS="$FPGA/gsensor_udp"
LD="$FPGA/ld19_udp"
EN="$FPGA/encoder_udp"
TOOL="$REPO/create_udp_packet/udp_packet.py"

mkdir -p "$SIM_DIR/build"
cd "$SIM_DIR/build"

# Expected Ethernet/IP/UDP headers: same fields as gsensor_frame.v, ld19_frame.v, enc_frame.v
COMMON=(--src-ip 192.168.17.200 --dst-ip 192.168.17.255 --ip-id 0 --ttl 64 --df --no-udp-checksum)
python3 "$TOOL" gen "${COMMON[@]}" --src-port 1024 --dst-port 1024 --payload-len 18 -o gs_hdr.bin > /dev/null
python3 "$TOOL" show gs_hdr.bin --hex expected_gs_hdr.hex > /dev/null
python3 "$TOOL" gen "${COMMON[@]}" --src-port 1025 --dst-port 1025 --payload-len 59 -o ld_hdr.bin > /dev/null
python3 "$TOOL" show ld_hdr.bin --hex expected_ld_hdr.hex > /dev/null
python3 "$TOOL" gen "${COMMON[@]}" --src-port 1026 --dst-port 1026 --payload-len 30 -o en_hdr.bin > /dev/null
python3 "$TOOL" show en_hdr.bin --hex expected_en_hdr.hex > /dev/null

# Example packet from the LD19 Development Manual, section 3.3
echo "54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9 CD 00 E4 CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50" \
    | tr ' ' '\n' > ld19_example.hex

RTL=("$PRJ/sensors_udp_top.v" "$EN/quad_decoder.v" "$EN/enc_frame.v"
     "$GS/adxl345_spi3.v" "$GS/gsensor_frame.v"
     "$LD/uart_rx.v" "$LD/ld19_packet.v" "$LD/ld19_frame.v"
     "$FPGA/udp_tx/eth_tx.v" "$FPGA/udp_tx/crc32.v")

status=0

iverilog -o tb_sensors_udp.vvp -s tb_sensors_udp "$SIM_DIR/tb_sensors_udp.v" "$GS/sim/adxl345_model.v" "${RTL[@]}"
vvp -n tb_sensors_udp.vvp "$@" | grep -v '^RAM READ\|^tx byte' | tee sim.log
grep -q '^PASS' sim.log || status=1

iverilog -DSENSORS_TOP -o tb_encoder_udp.vvp -s tb_encoder_udp "$EN/sim/tb_encoder_udp.v" "${RTL[@]}"
vvp -n tb_encoder_udp.vvp "$@" | grep -v '^RAM READ\|^tx byte' | tee sim_encoders.log
grep -q '^PASS' sim_encoders.log || status=1

exit $status
