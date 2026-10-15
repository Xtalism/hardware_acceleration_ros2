#!/usr/bin/env bash
# Run the self-checking testbench tb_ld19_udp.v with Icarus Verilog, with the
# LD19 at the exact baud rate and 2 % fast and slow.
#
#   ./run_sim.sh          run, prints PASS or FAIL for each baud rate
#   ./run_sim.sh +vcd     also dump build/tb_ld19_udp.vcd (open with gtkwave)
#
# Exit status 0 when all runs pass.
set -euo pipefail

SIM_DIR=$(cd "$(dirname "$0")" && pwd)
PRJ=$(dirname "$SIM_DIR")
FPGA=$(dirname "$PRJ")
REPO=$(dirname "$FPGA")

mkdir -p "$SIM_DIR/build"
cd "$SIM_DIR/build"

# Expected Ethernet/IP/UDP header: same fields as ld19_frame.v
python3 "$REPO/create_udp_packet/udp_packet.py" gen --src-ip 192.168.17.200 --dst-ip 192.168.17.255 \
    --src-port 1025 --dst-port 1025 --payload-len 59 --ip-id 0 --ttl 64 --df --no-udp-checksum \
    -o hdr.bin > /dev/null
python3 "$REPO/create_udp_packet/udp_packet.py" show hdr.bin --hex expected_hdr.hex > /dev/null

# Example packet from the LD19 Development Manual, section 3.3
echo "54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9 CD 00 E4 CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50" \
    | tr ' ' '\n' > ld19_example.hex

status=0
for ppm in 0 20000 -20000; do
    iverilog -o tb_ld19_udp.vvp -s tb_ld19_udp -P tb_ld19_udp.BAUD_ERR_PPM=$ppm \
        "$SIM_DIR/tb_ld19_udp.v" \
        "$PRJ/ld19_udp_top.v" "$PRJ/uart_rx.v" "$PRJ/ld19_packet.v" "$PRJ/ld19_frame.v" \
        "$FPGA/udp_tx/eth_tx.v" "$FPGA/udp_tx/crc32.v"
    vvp -n tb_ld19_udp.vvp "$@" | grep -v '^RAM READ\|^tx byte' | tee sim_$ppm.log
    grep -q '^PASS' sim_$ppm.log || status=1
done
exit $status
