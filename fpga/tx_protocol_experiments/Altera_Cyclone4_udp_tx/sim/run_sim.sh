#!/usr/bin/env bash
# Run the self-checking testbench tb_eth_tx.v.
#
#   ./run_sim.sh            Icarus Verilog (sudo apt install iverilog), RAM model
#   ./run_sim.sh questa     Questa-Intel FPGA Starter Edition, real altsyncram;
#                           needs SALT_LICENSE_SERVER pointing at your license.dat
#   ./run_sim.sh ... +vcd   also dump build/tb_eth_tx.vcd (open with gtkwave)
#
# Exit status 0 on PASS.
set -euo pipefail

SIM_DIR=$(cd "$(dirname "$0")" && pwd)
PRJ=$(dirname "$SIM_DIR")
TOOL=iverilog
if [[ $# -gt 0 && $1 != +* ]]; then
    TOOL=$1
    shift
fi
QUESTA_BIN=${QUESTA_BIN:-/opt/altera_lite/25.1std/questa_fse/bin}

mkdir -p "$SIM_DIR/build"
cd "$SIM_DIR/build"

# Expected frame bytes from the same MIF that initialises the RAM
python3 "$PRJ/../create_udp_packet/udp_packet.py" show "$PRJ/ram_ip_paket_tx.mif" --hex expected.hex

RTL=("$PRJ/crc32.v" "$PRJ/eth_tx.v" "$PRJ/top.v")

case "$TOOL" in
iverilog)
    iverilog -o tb_eth_tx.vvp -s tb_eth_tx \
        "$SIM_DIR/tb_eth_tx.v" "$SIM_DIR/ram_ip_paket_tx_model.v" "${RTL[@]}"
    vvp -n tb_eth_tx.vvp "$@" | tee sim.log
    ;;
questa)
    cp "$PRJ/ram_ip_paket_tx.mif" .   # altsyncram reads init_file from the run directory
    "$QUESTA_BIN/vlib" work
    "$QUESTA_BIN/vlog" -quiet -timescale 1ns/1ps \
        "$SIM_DIR/tb_eth_tx.v" "$PRJ/ram_ip_paket_tx.v" "${RTL[@]}"
    "$QUESTA_BIN/vsim" -c -quiet -onfinish exit -L altera_mf_ver work.tb_eth_tx "$@" \
        -do "run -all" | tee sim.log
    ;;
*)
    echo "usage: $0 [iverilog|questa] [+vcd]" >&2
    exit 2
    ;;
esac

grep -Eq '^(# )?PASS' sim.log
