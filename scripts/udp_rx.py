#!/usr/bin/env python3
"""Receive the FPGA's UDP test packets on Linux and check them.

The kernel only hands over datagrams whose FCS, IP and UDP checksums are
good, so every packet counted here made it through the whole stack.
Payload is checked against the 00 01 02 ... pattern from udp_packet.py.

  ./udp_rx.py                 listen on UDP port 1024 (the default MIF)
  ./udp_rx.py --port 10000 --no-check
"""
import argparse
import socket
import time

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument('--port', type=int, default=1024)
p.add_argument('--no-check', action='store_true', help='do not check the payload pattern')
a = p.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
sock.bind(('', a.port))  # also receives broadcasts such as 192.168.17.255
sock.settimeout(1.0)
print('listening on UDP port %d, Ctrl-C to stop' % a.port)

total = bad = 0
n = nbytes = 0
t0 = time.monotonic()
try:
    while True:
        try:
            data, src = sock.recvfrom(65535)
        except socket.timeout:
            data = None
        if data is not None:
            if total == 0:
                print('first packet from %s:%d, %d bytes: %s' % (src[0], src[1], len(data), data.hex(' ')))
            total += 1
            n += 1
            nbytes += len(data)
            if not a.no_check and data != bytes(i & 0xFF for i in range(len(data))):
                bad += 1
        now = time.monotonic()
        if now - t0 >= 1.0:
            print('%7.0f pkt/s  %8.3f MB/s payload  total %d  bad payload %d'
                  % (n / (now - t0), nbytes / (now - t0) / 1e6, total, bad))
            n = nbytes = 0
            t0 = now
except KeyboardInterrupt:
    print('\n%d packets, %d with bad payload' % (total, bad))
