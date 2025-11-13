import pyvisa
import numpy as np
import matplotlib.pyplot as plt
import plotly.express as px

RESOURCE = 'USB0::0x1AB1::0x044D::DHO8A272806711::INSTR'

rm = pyvisa.ResourceManager()
scope = rm.open_resource(RESOURCE,
                         timeout=20000,        # ms
                         chunk_size=10_000_000 # allow big transfers
                         )

scope.write('*CLS')

# Choose the source channel and data mode
scope.write(':WAV:SOUR CHAN1')   # CHAN1..CHAN4
scope.write(':WAV:MODE RAW')     # RAW = full memory, NORM = screen data
scope.write(':WAV:FORM BYTE')    # BYTE (0–255) is fastest; WORD possible too

# Ask for scaling parameters (preamble)
# Most Rigol scopes return: format,type,points,count,xinc,xorigin,xref,yinc,yorigin,yref
preamble = scope.query_ascii_values(':WAV:PRE?')
fmt, dtype, points, count, xinc, xorigin, xref, yinc, yorigin, yref = preamble

# Read the binary block
# Many Rigols use IEEE 488.2 definite-length blocks starting with '#'
scope.write(':WAV:DATA?')
raw = scope.read_raw()

if raw[0:1] != b'#':
    raise RuntimeError('Unexpected data header')

ndigits = int(raw[1:2])
nbytes = int(raw[2:2+ndigits])
payload = raw[2+ndigits:2+ndigits+nbytes]

# Convert to numpy
y_u8 = np.frombuffer(payload, dtype=np.uint8)

i = np.arange(points, dtype=np.float64)

print(preamble)

# Plot
plt.figure()
plt.plot(i, y_u8)
plt.xlabel('i')
plt.ylabel('y_u8')
plt.title('Rigol DHO804 – CH1 (RAW memory)')
plt.grid(True)
plt.show()

scope.close()
rm.close()