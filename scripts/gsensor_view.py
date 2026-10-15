#!/usr/bin/env python3
"""Live view of the DE0-Nano ADXL345 samples sent over UDP by
Altera_Cyclone4_gsensor_udp (one packet per sample, 18-byte payload).

  ./gsensor_view.py                 live values, bars and stats in the terminal
  ./gsensor_view.py --plot          ... plus a scrolling graph window (matplotlib)
  ./gsensor_view.py --csv log.csv   also save every sample
"""
import argparse
import collections
import math
import select
import shutil
import socket
import struct
import sys
import time

PAYLOAD = struct.Struct('>2sIIhhhBB')   # "GS", seq, time_us, x, y, z, devid, flags
G_PER_LSB = 1 / 256                     # ADXL345 full resolution, 3.9 mg/LSB
DEVID_OK = 0xE5


class Receiver:
    def __init__(self, port, keep_s, csv_path=None):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(('', port))       # also receives broadcasts (192.168.17.255)
        self.sock.setblocking(False)

        n = int(keep_s * 3200)           # enough for the ADXL345's fastest rate
        self.t = collections.deque(maxlen=n)
        self.x = collections.deque(maxlen=n)
        self.y = collections.deque(maxlen=n)
        self.z = collections.deque(maxlen=n)

        self.total = self.lost = self.dropped = self.bad = 0
        self.src = None
        self.seq = None
        self.devid = None
        self.t_us = None
        self.t_s = 0.0
        self.last_rx = None
        self.rate = 0.0
        self._rate_n = 0
        self._rate_t0 = time.monotonic()
        self.csv = open(csv_path, 'w') if csv_path else None
        if self.csv:
            self.csv.write('seq,time_s,x_g,y_g,z_g\n')

    def poll(self):
        while True:
            try:
                data, src = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            self._handle(data, src)
        now = time.monotonic()
        if now - self._rate_t0 >= 1.0:
            self.rate = self._rate_n / (now - self._rate_t0)
            self._rate_n = 0
            self._rate_t0 = now

    def _handle(self, data, src):
        if len(data) != PAYLOAD.size or data[:2] != b'GS':
            self.bad += 1
            return
        _, seq, t_us, x, y, z, devid, flags = PAYLOAD.unpack(data)

        restarted = self.seq is None or ((seq - self.seq) & 0xFFFFFFFF) > 1_000_000
        if restarted:
            self.t_s = 0.0
        else:
            self.lost += (seq - self.seq - 1) & 0xFFFFFFFF
            self.t_s += ((t_us - self.t_us) & 0xFFFFFFFF) / 1e6

        self.seq, self.t_us, self.devid, self.src = seq, t_us, devid, src[0]
        self.dropped += flags & 1
        self.total += 1
        self._rate_n += 1
        self.last_rx = time.monotonic()

        xg, yg, zg = x * G_PER_LSB, y * G_PER_LSB, z * G_PER_LSB
        self.t.append(self.t_s)
        self.x.append(xg)
        self.y.append(yg)
        self.z.append(zg)
        if self.csv:
            self.csv.write('%d,%.6f,%.4f,%.4f,%.4f\n' % (seq, self.t_s, xg, yg, zg))


def bar(v, full, width=41):
    half = width // 2
    cells = ['-'] * width
    cells[half] = '|'
    cells[half + round(max(-1.0, min(1.0, v / full)) * half)] = '#'
    return ''.join(cells)


class Display:
    """Redraws a fixed block of lines in place (or prints a line a second if not a terminal)."""

    def __init__(self, rx, g_range):
        self.rx = rx
        self.g_range = g_range
        self.tty = sys.stdout.isatty()
        self.drawn = 0
        self.t_next = 0.0
        self.t_start = time.monotonic()

    def lines(self):
        rx = self.rx
        now = time.monotonic()
        if rx.total == 0:
            out = ['waiting for G-sensor packets on UDP port %d ... (%.0f s)' % (rx.port, now - self.t_start)]
            if rx.bad:
                out.append('%d packets on this port are not G-sensor packets: is the old test-pattern design loaded?' % rx.bad)
            elif now - self.t_start > 3:
                out.append('nothing yet: check the link (ethtool), the fpga profile (192.168.17.1) and that the board is programmed')
            return out

        x, y, z = rx.x[-1], rx.y[-1], rx.z[-1]
        a = math.sqrt(x * x + y * y + z * z)
        pitch = math.degrees(math.atan2(-x, math.sqrt(y * y + z * z)))
        roll = math.degrees(math.atan2(y, z))
        out = [
            'ADXL345 %s  seq %d  %.1f/s  lost %d  dropped %d  DEVID %02X'
            % (rx.src, rx.seq, rx.rate, rx.lost, rx.dropped, rx.devid),
            '  X %+7.3f g  [%s]' % (x, bar(x, self.g_range)),
            '  Y %+7.3f g  [%s]' % (y, bar(y, self.g_range)),
            '  Z %+7.3f g  [%s]' % (z, bar(z, self.g_range)),
            ' |a| %6.3f g  pitch %+6.1f  roll %+6.1f deg  t %.1f s' % (a, pitch, roll, rx.t_s),
        ]
        if rx.devid != DEVID_OK:
            out.append('WARNING: ADXL345 DEVID is %02X, not E5: the sensor is not answering' % rx.devid)
        elif now - rx.last_rx > 1.0:
            out.append('WARNING: no packets for %.0f s' % (now - rx.last_rx))
        elif rx.bad:
            out.append('%d packets ignored (not G-sensor format)' % rx.bad)
        return out

    def update(self, force=False):
        now = time.monotonic()
        if not force and now < self.t_next:
            return
        if self.tty:
            self.t_next = now + 0.1
            # clip to the terminal width: a wrapped line would break the redraw
            width = shutil.get_terminal_size().columns - 1
            lines = [ln[:width] for ln in self.lines()]
            if self.drawn:
                sys.stdout.write('\x1b[%dF' % self.drawn)
            for ln in lines + [''] * (self.drawn - len(lines)):
                sys.stdout.write('\x1b[2K' + ln + '\n')
            self.drawn = max(self.drawn, len(lines))
        else:
            self.t_next = now + 1.0
            sys.stdout.write(' | '.join(s.strip() for s in self.lines()) + '\n')
        sys.stdout.flush()


def run_terminal(rx, disp, duration):
    t_end = time.monotonic() + duration if duration else None
    while t_end is None or time.monotonic() < t_end:
        select.select([rx.sock], [], [], 0.05)
        rx.poll()
        disp.update()


def run_plot(rx, disp, window_s, g_range):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots(figsize=(10, 5))
    fig.canvas.manager.set_window_title('ADXL345 over UDP')
    lines = [ax.plot([], [], lw=1.2, label=name)[0] for name in ('X', 'Y', 'Z')]
    ax.set_xlim(-window_s, 0)
    ax.set_ylim(-g_range, g_range)
    ax.set_xlabel('time (s, 0 = newest sample)')
    ax.set_ylabel('acceleration (g)')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper left')
    title = ax.set_title('waiting for packets on UDP port %d ...' % rx.port)

    def update(_):
        rx.poll()
        disp.update()
        if rx.t:
            t_new = rx.t[-1]
            # only the samples inside the window
            i = 0
            while i < len(rx.t) and rx.t[i] < t_new - window_s:
                i += 1
            t = [v - t_new for v in list(rx.t)[i:]]
            for ln, d in zip(lines, (rx.x, rx.y, rx.z)):
                ln.set_data(t, list(d)[i:])
            title.set_text('%.0f samples/s   lost %d   X %+.3f  Y %+.3f  Z %+.3f g'
                           % (rx.rate, rx.lost, rx.x[-1], rx.y[-1], rx.z[-1]))
        return lines + [title]

    anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.show()
    return anim


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', type=int, default=1024)
    p.add_argument('--plot', action='store_true', help='open a live graph window')
    p.add_argument('--window', type=float, default=5.0, help='graph time window in seconds (default 5)')
    p.add_argument('--range', type=float, default=2.0, help='+- g shown on bars and graph (default 2)')
    p.add_argument('--csv', help='save every sample to this CSV file')
    p.add_argument('--duration', type=float, help='stop after this many seconds')
    a = p.parse_args()

    rx = Receiver(a.port, max(a.window, 1.0) + 1.0, a.csv)
    disp = Display(rx, a.range)
    try:
        if a.plot:
            run_plot(rx, disp, a.window, a.range)
        else:
            run_terminal(rx, disp, a.duration)
    except KeyboardInterrupt:
        pass
    finally:
        disp.update(force=True)
        print('\n%d samples, %d lost, %d dropped in the FPGA, %d other packets'
              % (rx.total, rx.lost, rx.dropped, rx.bad))
        if rx.csv:
            rx.csv.close()


if __name__ == '__main__':
    main()
