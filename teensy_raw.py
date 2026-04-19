#!/usr/bin/env python3
"""Raw serial terminal for Teensy — see what firmware is actually running."""
import sys, serial, threading, time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"

ser = serial.Serial(PORT, 115200, timeout=0.1)
time.sleep(0.5)
ser.reset_input_buffer()

def reader():
    while True:
        line = ser.readline()
        if line:
            print(f"  << {line.decode('utf-8', errors='replace').rstrip()}")

t = threading.Thread(target=reader, daemon=True)
t.start()

print(f"Connected to {PORT}. Type raw commands (Ctrl-C to quit).")
print("Try: HELP  STATUS  HOME  STREAM 10  MOVE 11 90\n")

while True:
    try:
        cmd = input("> ").strip()
        if cmd:
            ser.write((cmd + "\n").encode())
            ser.flush()
            time.sleep(0.1)
    except (KeyboardInterrupt, EOFError):
        break

ser.close()
