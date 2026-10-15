#!/usr/bin/env bash
# Run the self-checking testbench tb_gsensor_udp.v with Icarus Verilog.
#
#   ./run_sim.sh          run, prints PASS or FAIL
#   ./run_sim.sh +vcd     also dump build/tb_gsensor_udp.vcd (open with gtkwave)
#
# Exit status 0 on PASS.
set -euo pipefail

SIM_DIR=$(cd "$(dirname "$0")" && pwd)
PRJ=$(dirname "$SIM_DIR")
FPGA=$(dirname "$PRJ")
REPO=$(dirname "$FPGA")

mkdir -p "$SIM_DIR/build"
cd "$SIM_DIR/build"

# Expected Ethernet/IP/UDP header: same fields as gsensor_frame.v
python3 "$REPO/create_udp_packet/udp_packet.py" gen --src-ip 192.168.17.200 --dst-ip 192.168.17.255 \
    --src-port 1024 --dst-port 1024 --payload-len 18 --ip-id 0 --ttl 64 --df --no-udp-checksum \
    -o hdr.bin > /dev/null
python3 "$REPO/create_udp_packet/udp_packet.py" show hdr.bin --hex expected_hdr.hex > /dev/null

iverilog -o tb_gsensor_udp.vvp -s tb_gsensor_udp \
    "$SIM_DIR/tb_gsensor_udp.v" "$SIM_DIR/adxl345_model.v" \
    "$PRJ/gsensor_udp_top.v" "$PRJ/adxl345_spi3.v" "$PRJ/gsensor_frame.v" \
    "$FPGA/udp_tx/eth_tx.v" "$FPGA/udp_tx/crc32.v"
vvp -n tb_gsensor_udp.vvp "$@" | grep -v '^RAM READ\|^tx byte' | tee sim.log

grep -q '^PASS' sim.log
