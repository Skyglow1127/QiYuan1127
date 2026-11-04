#!/usr/bin/env python3
"""Passive D1 CP2102N RX only. Never opens the Xilinx PPP serial devices.

No writes, BREAK, flush, or explicit DTR/RTS operations. Serial line coding is
temporarily selected for observation and original termios settings restored.
Opening a TTY may still cause driver-managed modem-line behavior.
"""
import argparse
import copy
import fcntl
import json
import os
from pathlib import Path
import select
import termios
import time

DEVICE = Path('/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_5a82abe804c6ef11a7a296e78048b910-if00-port0')
PLAN = [(115200,20), (1500000,15), (921600,10), (460800,5),
        (230400,5), (57600,10), (9600,10)]


def run(output):
    target = DEVICE.resolve(strict=True)
    sys = Path('/sys/class/tty')/target.name/'device'
    parents = [sys.resolve(), *sys.resolve().parents]
    usb = next((p for p in parents if (p/'idVendor').exists()), None)
    if usb is None or (usb/'idVendor').read_text().strip()!='10c4' or (usb/'idProduct').read_text().strip()!='ea60':
        raise RuntimeError('Not the expected D1 CP210x USB device; refused')
    if (usb/'serial').read_text().strip()!='5a82abe804c6ef11a7a296e78048b910':
        raise RuntimeError('Unexpected USB serial identity; refused')
    output.mkdir(parents=True, exist_ok=False)
    result = dict(device=str(DEVICE), resolved=str(target), bytes_written=0,
                  explicit_modem_line_operations=False, break_sent=False,
                  xilinx_ports_opened=False, restored=False, samples=[])
    fd = os.open(DEVICE, os.O_RDONLY|os.O_NOCTTY|os.O_NONBLOCK)
    original = None
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        original = termios.tcgetattr(fd)
        result['original_speed_constant'] = original[4]
        result['original_hupcl'] = bool(original[2]&termios.HUPCL)
        for baud, seconds in PLAN:
            speed = getattr(termios, 'B'+str(baud), None)
            if speed is None:
                result['samples'].append(dict(baud=baud, skipped='platform constant unavailable'))
                continue
            a = copy.deepcopy(original)
            a[0] = 0; a[1] = 0; a[3] = 0
            a[2] = (a[2] & ~(termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.CRTSCTS))|termios.CS8|termios.CLOCAL|termios.CREAD
            a[4] = a[5] = speed
            a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, a)
            chunks=[]; started=time.monotonic()
            while time.monotonic()-started < seconds:
                ready,_,_=select.select([fd],[],[],min(.25,max(.001,seconds-(time.monotonic()-started))))
                if ready:
                    data=os.read(fd,65536)
                    if data: chunks.append(data)
                if sum(map(len,chunks))>1048576:
                    raise RuntimeError('Capture size bound reached')
            raw=b''.join(chunks)
            (output/f'rx_{baud}.bin').write_bytes(raw)
            (output/f'rx_{baud}.txt').write_text(raw.decode('utf-8',errors='replace'))
            item=dict(baud=baud,seconds=time.monotonic()-started,bytes=len(raw))
            result['samples'].append(item)
            print(json.dumps(item),flush=True)
    finally:
        try:
            if original is not None:
                termios.tcsetattr(fd,termios.TCSANOW,original)
                result['restored'] = termios.tcgetattr(fd)==original
        finally:
            os.close(fd)
            (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.execute: run(args.output)
    else: print('Plan only; RX on fixed CP2102N identity:',PLAN)
