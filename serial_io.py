"""
serial_io.py — Arduino serial communication
"""

import time
import serial
import serial.tools.list_ports

from config import SERIAL_PORT, BAUD


def find_arduino():
    """Scan serial ports and return the first that looks like an Arduino."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if any(k in desc for k in ("arduino", "ch340", "cp210", "ftdi")):
            return p.device
    return SERIAL_PORT


def open_serial(port):
    """Open the serial port and wait for the Arduino READY handshake."""
    try:
        ser      = serial.Serial(port, BAUD, timeout=1)
        deadline = time.time() + 3.0
        while time.time() < deadline:
            if ser.readline().decode(errors="ignore").strip() == "READY":
                print(f"[serial] Arduino ready on {port}")
                return ser
        print(f"[serial] no READY from {port} — continuing anyway")
        return ser
    except serial.SerialException as e:
        print(f"[serial] {e} — running without Arduino")
        return None


def send_velocity(ser, vel):
    """Send a signed integer velocity (steps/sec) to the Arduino."""
    if ser:
        ser.write(f"{int(vel)}\n".encode())
