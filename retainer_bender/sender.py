"""Stream a program to the bender over USB serial, one line at a time."""

import sys
import time


def program_lines(path):
    for raw in open(path):
        line = raw.split(";", 1)[0].strip()
        if line:
            yield line


def send(path, port, baud=115200, home=True, dry_run=False, timeout=60.0):
    lines = (["HOME"] if home else []) + list(program_lines(path))
    if dry_run:
        for line in lines:
            print(line)
        return 0
    try:
        import serial
    except ImportError:
        raise SystemExit("pyserial is not installed: pip install pyserial")
    with serial.Serial(port, baud, timeout=0.2) as ser:
        time.sleep(2.0)                 # most boards reset when the port opens
        ser.reset_input_buffer()
        for line in lines:
            ser.write((line + "\n").encode())
            deadline = time.time() + timeout
            while True:
                reply = ser.readline().decode(errors="replace").strip()
                if reply == "ok":
                    print(f"{line:<20} ok")
                    break
                if reply.startswith("error"):
                    print(f"{line:<20} {reply}", file=sys.stderr)
                    ser.write(b"ARM 0\n")
                    return 1
                if reply:
                    print(f"  {reply}")
                if time.time() > deadline:
                    print(f"{line:<20} no reply in {timeout:.0f} s; stopping", file=sys.stderr)
                    return 1
    return 0
