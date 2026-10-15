#!/usr/bin/env bash
# Run the self-checking testbench tb_encoder_udp.v (four quadrature encoders)
# against encoder_udp_top with Icarus Verilog.
#
#   ./run_sim.sh          run, prints PASS or FAIL
#   ./run_sim.sh +vcd     also dump build/tb_encoder_udp.vcd (open with gtkwave)
#
# Exit status 0 on PASS.
set -euo pipefail

SIM_DIR=$(cd "$(dirname "$0")" && pwd)
PRJ=$(dirname "$SIM_DIR")
FPGA=$(dirname "$PRJ")
REPO=$(dirname "$FPGA")

mkdir -p "$SIM_DIR/build"
cd "$SIM_DIR/build"

# Expected Ethernet/IP/UDP header: same fields as enc_frame.v
python3 "$REPO/create_udp_packet/udp_packet.py" gen --src-ip 192.168.17.200 --dst-ip 192.168.17.255 \
    --src-port 1026 --dst-port 1026 --payload-len 30 --ip-id 0 --ttl 64 --df --no-udp-checksum \
    -o en_hdr.bin > /dev/null
python3 "$REPO/create_udp_packet/udp_packet.py" show en_hdr.bin --hex expected_en_hdr.hex > /dev/null

iverilog -o tb_encoder_udp.vvp -s tb_encoder_udp \
    "$SIM_DIR/tb_encoder_udp.v" \
    "$PRJ/encoder_udp_top.v" "$PRJ/quad_decoder.v" "$PRJ/enc_frame.v" \
    "$FPGA/udp_tx/eth_tx.v" "$FPGA/udp_tx/crc32.v"
vvp -n tb_encoder_udp.vvp "$@" | grep -v '^RAM READ\|^tx byte' | tee sim.log

grep -q '^PASS' sim.log
