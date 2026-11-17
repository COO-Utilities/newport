# smc100pp_controller

Low-level Python modules to send commands to Newport motion controllers.

## Currently Supported Models
- SMC100PP stage controller - smc100pp.py
- Oriel Cornerstone 260 monochromator - cornerstone260.py

## SMC100PP

### Features
- Connect to Newport controllers over serial through a terminal server
- Query stage state and parameters
- Move individual axes to absolute or relative positions

### Usage

```python
import smc100pp

controller = smc100pp.StageController()
controller.connect(host='192.168.29.100', port=10006)

# Print stage 1 parameters
print(controller.get_params(1))

# Print stage2 state
print(controller.get_state(2))

# Move axis 1 to position 12.0
controller.move_abs(12.0, 1)

# Move axis 2 to +12 degrees relative to current position
controller.move_rel(12.0, 2)

# For a comprehensive list of classes and methods, use the help function
help(smc100pp)

```

## Cornerstone 260

### Features
- Connect to the monochromator over a directly attached RS-232 port at 9600 8N1
- Move the wavelength drive, select gratings and change units
- Control the shutter, filter wheel, motorized slits and output port
- Acknowledge completion of every statement through the instrument's handshake mode

### Connect and identify

The monochromator is RS-232 at 9600 8N1 with no flow control, on a directly attached
serial port.

```python
from cornerstone260 import Cornerstone260

mono = Cornerstone260(log=False)   # log=True writes cornerstone260.log in the working directory
mono.connect("/dev/ttyS0")         # baud defaults to 9600
mono.initialize()                  # turn on handshake mode; returns False if it did not take
print(mono.info())
# InstrumentInfo(vendor='Oriel Instruments', model='Cornerstone 260',
#                serial_number='281', firmware_version='03.10', raw='...')
```

- `connect()` raises `IOError` if the port cannot be opened, for example when another
  program (a terminal, or a daemon) already holds it.
- Always call `initialize()`. Handshake mode is what makes every move block until it has
  finished, and what turns instrument errors into exceptions.
- After a power cycle, give the controller up to a minute to home its drives before
  sending anything.
- `mono.disconnect()` closes the port when you are done.

### Wavelength

```python
from cornerstone260 import Units

mono.get_units()              # Units.NANOMETERS
mono.get_wavelength()         # 500.0, in the current units
mono.set_wavelength(550.0)    # blocks until the drive arrives
mono.set_units(Units.MICROMETERS)

mono.get_step()               # drive position in motor steps from the home sensor
mono.step(10)                 # move by a signed number of steps
mono.abort()                  # stop wavelength motion
```

- Units are `Units.NANOMETERS`, `Units.MICROMETERS` or `Units.WAVENUMBERS`, and apply to
  both moves and readings.
- A wavelength the selected grating cannot reach is rejected with error 3.
- Near 0 the grating acts as a mirror (zero order), so white light comes out.
- `calibrate(wavelength)` redefines the current position as that wavelength and stores the
  offset in the instrument, so only use it against a known reference line.

### Gratings

```python
mono.get_grating()            # GratingInfo(number=2, lines_per_mm=600, label='1000')
mono.set_grating(1)           # rotate the turret; 1, 2 or 3
mono.get_grating_lines(3)     # groove density of any grating
mono.get_grating_label(3)
```

The maximum wavelength scales inversely with groove density, about 1600 nm at 1200 l/mm,
and `get_limits()` reports it for the selected grating. The factor, offset and zero
setters change the calibration the instrument moves by; leave them alone unless you are
recalibrating.

### Shutter and output port

```python
from cornerstone260 import OutputPort, ShutterState

mono.get_shutter()            # ShutterState.OPEN or ShutterState.CLOSED
mono.close_shutter()
mono.open_shutter()

mono.get_output_port()        # OutputPort.AXIAL or OutputPort.LATERAL
mono.set_output_port(OutputPort.AXIAL)   # move the flip mirror
```

### Filter wheel

```python
mono.get_filter()             # position 1 to 6
mono.set_filter(2)
mono.get_filter_label(2)
```

> **Check the wheel turns freely before commanding it.** A jammed wheel keeps its motor
> energized, which has overheated a wheel cable. For a first move, go to the adjacent
> position, watch the wheel and be ready to cut power; `abort()` stops only the
> wavelength drive.

- A missing wheel should answer `0` with error 6, which `get_filter()` reports as a
  warning and `None`. A fitted but faulty wheel answered `0` with error 0 on SN281, which
  raises `CornerstoneError`.
- A faulty wheel can also stall the whole controller: statements are echoed but answered
  late or not at all. Unplug the wheel and power cycle to confirm.

### Errors

| Exception | Raised when |
|---|---|
| `IOError` | `connect()` cannot open the serial port |
| `ValueError` | A grating, filter, slit or label argument is out of range; nothing is sent |
| `CornerstoneError` | The instrument reports an error, or its reply is late or garbled |

The `CornerstoneError` message carries the instrument's error code:

| Code | Meaning |
|---|---|
| 0 | System error (miscellaneous) |
| 1 | Command not understood |
| 2 | Bad parameter |
| 3 | Wavelength not allowed |
| 6 | Accessory not present |
| 7 | Accessory already in position |
| 8 | Could not home wavelength drive |
| 9 | Label too long |

When a reply does not match the statement sent, the driver drains the stale input and
raises rather than return the wrong value, so retry. `get_error_code()` reads and clears
a pending error, and `has_error()` reads and clears the status byte.

### Talking to it by hand

Any serial terminal works, as long as it sends a linefeed:

```bash
python -m serial.tools.miniterm /dev/ttyS0 9600 --eol LF
```

The instrument echoes each statement before replying, so a terminal shows both. Queries
end in `?` and only read (`INFO?`, `WAVE?`, `STEP?`, `GRAT?`, `SHUTTER?`, `OUTPORT?`,
`FILTER?`, `ERROR?`, `STB?`); everything else moves or changes something. Close the
terminal before connecting from Python, since only one program can hold the port.

### Quick reference

| Method | Statement |
|---|---|
| `info()` | `INFO?` |
| `get_wavelength()` / `set_wavelength(w)` | `WAVE?` / `GOWAVE w` |
| `get_step()` / `step(n)` | `STEP?` / `STEP n` |
| `abort()` | `ABORT` |
| `calibrate(w)` | `CALIBRATE w` |
| `get_units()` / `set_units(u)` | `UNITS?` / `UNITS u` |
| `get_grating()` / `set_grating(n)` | `GRAT?` / `GRAT n` |
| `get_grating_lines(n)` / `get_grating_label(n)` | `GRATnLINES?` / `GRATnLABEL?` |
| `get_shutter()` / `open_shutter()` / `close_shutter()` | `SHUTTER?` / `SHUTTER O` / `SHUTTER C` |
| `get_output_port()` / `set_output_port(p)` | `OUTPORT?` / `OUTPORT p` |
| `get_filter()` / `set_filter(n)` | `FILTER?` / `FILTER n` |
| `get_filter_label(n)` | `FILTERnLABEL?` |
| `get_slit_microns(n)` / `set_slit_microns(n, um)` | `SLITnMICRONS?` / `SLITnMICRONS um` |
| `get_bandpass()` / `set_bandpass(bp)` | `BANDPASS?` / `BANDPASS bp` |
| `get_handshake()` / `set_handshake(on)` | `HANDSHAKE?` / `HANDSHAKE 1` |
| `get_error_code()` | `ERROR?` |
| `get_status_byte()` / `has_error()` | `STB?` |
| `get_limits()` | per-axis ranges for the installed accessories |

`help(cornerstone260)` lists everything.

### Firmware notes

The status byte has two encodings. The handshake acknowledgement is decimal, `0` for
success and `32` for an error, while `STB?` reports the same byte in hex, `00` and `20`.
Manual §16.7 quotes both numbers without distinguishing them.

Other places firmware V03.10 departs from the Rev A manual: responses are not all upper
case (`UNITS?` answers `nm`), `INFO?` carries neither the model number nor the documented
field layout, and `ERROR?` returns an empty line rather than `0` when nothing is pending.

## 🧪 Testing
Unit tests are located in `tests/` directory.

To run all tests from the project root:

```bash
pytest
```