import time
import struct
import serial
import matplotlib.pyplot as plt
import numpy as np

PORT = "COM6"          

N  = 8192
fs = 4.3e6
BYTES_NEEDED = 4 * N   # 4 bytes per float

with serial.Serial(PORT, 115200, timeout=10) as ser:
    # GIGA resets when serial opens; give it a moment
    time.sleep(2)

    ser.reset_input_buffer()

    # Ask for data
    ser.write(b"G")

    # Read exactly 8192 floats (32768 bytes)
    raw = ser.read(BYTES_NEEDED)
    if len(raw) != BYTES_NEEDED:
        raise RuntimeError(f"Expected {BYTES_NEEDED} bytes, got {len(raw)}")

    # GIGA (ARM Cortex-M) is little-endian; so use '<' here
    floats = struct.unpack("<{}f".format(N), raw)

    # Test: print first 10 values
    print("First 10 floats:", floats[:10])


i = np.arange(0, len(floats), 1)

freq = np.linspace(0, fs/2, len(floats))

plt.figure()
plt.plot(freq / 1e6, floats)
plt.xlabel("f [MHz]")
plt.ylabel("A [V]")
plt.show()
