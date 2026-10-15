#!/usr/bin/env python3
"""Build and inspect the Ethernet/IPv4/UDP frame that eth_tx sends from RAM.

Linux replacement for the Windows tools in this repo:
  gen   generator_udp_packet.exe + bin2mif.exe: build a frame, write .mif/.bin
  show  EthernetUDP.exe: decode a .mif/.bin, verify checksums, print the bytes
        on the wire (preamble + frame + FCS); --hex writes a file for the testbench

Examples:
  ./udp_packet.py show ../Altera_Cyclone4_udp_tx/ram_ip_paket_tx.mif
  ./udp_packet.py gen --dst-ip 192.168.17.1 --payload-len 64 -o packet.mif
"""
import argparse
import ipaddress
import re
import struct
import sys
import zlib

MIN_FRAME = 60  # without FCS


def checksum16(data):
    if len(data) % 2:
        data += b'\0'
    s = sum(struct.unpack('!%dH' % (len(data) // 2), data))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return ~s & 0xFFFF


def mac(text):
    b = bytes.fromhex(text.replace(':', '').replace('-', ''))
    if len(b) != 6:
        raise argparse.ArgumentTypeError('MAC must be 6 bytes: %r' % text)
    return b


def ipv4(text):
    return ipaddress.IPv4Address(text).packed


def build_frame(a):
    payload = bytes(i & 0xFF for i in range(a.payload_len))
    udp_len = 8 + len(payload)
    flags = 0x4000 if a.df else 0
    ip = struct.pack('!BBHHHBBH4s4s', 0x45, 0, 20 + udp_len, a.ip_id, flags,
                     a.ttl, 17, 0, a.src_ip, a.dst_ip)
    ip = ip[:10] + struct.pack('!H', checksum16(ip)) + ip[12:]
    udp = struct.pack('!HHHH', a.src_port, a.dst_port, udp_len, 0) + payload
    if not a.no_udp_checksum:
        pseudo = a.src_ip + a.dst_ip + struct.pack('!BBH', 0, 17, udp_len)
        csum = checksum16(pseudo + udp) or 0xFFFF
        udp = udp[:6] + struct.pack('!H', csum) + udp[8:]
    frame = a.dst_mac + a.src_mac + b'\x08\x00' + ip + udp
    return frame.ljust(MIN_FRAME, b'\0')


def read_mif(path):
    text = open(path, encoding='ascii', errors='replace').read()
    text = re.sub(r'%[^%]*%', ' ', text)
    text = re.sub(r'--[^\n]*', ' ', text)
    for key in ('ADDRESS_RADIX', 'DATA_RADIX'):
        m = re.search(key + r'\s*=\s*(\w+)', text)
        if m and m.group(1).upper() != 'HEX':
            sys.exit('%s: only %s = HEX is supported' % (path, key))
    body = re.search(r'CONTENT\s+BEGIN(.*)END\s*;', text, re.S)
    if not body:
        sys.exit('%s: no CONTENT BEGIN ... END; block' % path)
    mem = {}
    for stmt in body.group(1).split(';'):
        stmt = stmt.strip()
        if not stmt:
            continue
        m = re.fullmatch(r'([0-9A-Fa-f]+)\s*:\s*([0-9A-Fa-f]+)', stmt)
        if not m:
            sys.exit('%s: unsupported MIF entry %r' % (path, stmt))
        mem[int(m.group(1), 16)] = int(m.group(2), 16)
    if sorted(mem) != list(range(len(mem))):
        sys.exit('%s: addresses are not contiguous from 0' % path)
    return bytes(mem[i] for i in range(len(mem)))


def write_mif(path, frame, comment):
    with open(path, 'w') as f:
        f.write('%% %s %%\n' % comment)
        f.write('DEPTH = %d;\nWIDTH = 8;\nADDRESS_RADIX = HEX;\nDATA_RADIX = HEX;\n'
                'CONTENT\nBEGIN\n' % len(frame))
        for i, b in enumerate(frame):
            f.write('%04X : %02X;\n' % (i, b))
        f.write('END;\n')


def fcs_bytes(frame):
    # CRC-32 as it goes on the wire: least significant byte first
    return struct.pack('<I', zlib.crc32(frame))


def describe(frame):
    ok = lambda good: 'OK' if good else 'BAD'
    out = ['frame: %d bytes + 4 FCS  ->  set i_ram_data_size to %d in top.v'
           % (len(frame), len(frame))]
    if len(frame) < MIN_FRAME:
        out.append('  WARNING: shorter than %d bytes, pad it (runt frame)' % MIN_FRAME)
    if len(frame) > 1500:
        out.append('  WARNING: eth_tx ignores i_tx_en when i_ram_data_size > PAKET_MAX_SIZE '
                   '(1500 in top.v), raise it to %d' % len(frame))
    out.append('  eth  dst %s  src %s  type %04x' % (
        frame[0:6].hex(':'), frame[6:12].hex(':'), struct.unpack('!H', frame[12:14])[0]))
    if frame[12:14] == b'\x08\x00' and len(frame) >= 34:
        ip = frame[14:34]
        total, = struct.unpack('!H', ip[2:4])
        out.append('  ip   %s -> %s  len %d  ttl %d  proto %d  checksum %04x %s' % (
            ipaddress.IPv4Address(ip[12:16]), ipaddress.IPv4Address(ip[16:20]), total,
            ip[8], ip[9], struct.unpack('!H', ip[10:12])[0], ok(checksum16(ip) == 0)))
        if ip[9] == 17 and len(frame) >= 42:
            sport, dport, ulen, ucsum = struct.unpack('!HHHH', frame[34:42])
            udp = frame[34:34 + ulen]
            if ucsum == 0:
                state = '(not used)'
            else:
                pseudo = ip[12:20] + struct.pack('!BBH', 0, 17, ulen)
                state = ok(len(udp) == ulen and checksum16(pseudo + udp) == 0)
            out.append('  udp  port %d -> %d  len %d (payload %d)  checksum %04x %s' % (
                sport, dport, ulen, ulen - 8, ucsum, state))
            if 20 + ulen != total or 14 + total > len(frame):
                out.append('  WARNING: IP/UDP length fields do not match the frame size')
    out.append('  FCS on the wire: %s' % fcs_bytes(frame).hex(' '))
    out.append('wire bytes (preamble, SFD, frame, FCS):')
    out.append('  ' + ('55' * 7 + 'd5' + frame.hex() + fcs_bytes(frame).hex()).upper())
    return '\n'.join(out)


def cmd_gen(a):
    frame = build_frame(a)
    if a.out.lower().endswith('.bin'):
        open(a.out, 'wb').write(frame)
    else:
        write_mif(a.out, frame, 'UDP %s:%d -> %s:%d' % (
            ipaddress.IPv4Address(a.src_ip), a.src_port,
            ipaddress.IPv4Address(a.dst_ip), a.dst_port))
    print('wrote %s' % a.out)
    print(describe(frame))


def cmd_show(a):
    frame = open(a.file, 'rb').read() if a.file.lower().endswith('.bin') else read_mif(a.file)
    print(describe(frame))
    if a.hex:
        with open(a.hex, 'w') as f:
            f.write(''.join('%02x\n' % b for b in frame))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    g = sub.add_parser('gen', help='build a frame and write .mif (or .bin)')
    # defaults reproduce Altera_Cyclone4_udp_tx/ram_ip_paket_tx.mif
    g.add_argument('--dst-mac', type=mac, default='ff:ff:ff:ff:ff:ff')
    g.add_argument('--src-mac', type=mac, default='00:12:34:56:78:90')
    g.add_argument('--src-ip', type=ipv4, default='192.168.17.200')
    g.add_argument('--dst-ip', type=ipv4, default='192.168.17.255')
    g.add_argument('--src-port', type=int, default=1024)
    g.add_argument('--dst-port', type=int, default=1024)
    g.add_argument('--payload-len', type=int, default=18, help='payload bytes 00 01 02 ...')
    g.add_argument('--ip-id', type=lambda s: int(s, 0), default=0xB3FE)
    g.add_argument('--ttl', type=int, default=128)
    g.add_argument('--df', action='store_true', help="set IP don't-fragment flag")
    g.add_argument('--no-udp-checksum', action='store_true', help='send UDP checksum 0')
    g.add_argument('-o', '--out', required=True, help='output .mif or .bin')
    g.set_defaults(func=cmd_gen)

    s = sub.add_parser('show', help='decode and check a .mif or .bin frame')
    s.add_argument('file')
    s.add_argument('--hex', help='also write one byte per line (testbench $fscanf input)')
    s.set_defaults(func=cmd_show)

    a = p.parse_args()
    if a.cmd == 'gen':
        if not 0 <= a.payload_len <= 1472:
            p.error('--payload-len must be 0..1472')
    a.func(a)


if __name__ == '__main__':
    main()
