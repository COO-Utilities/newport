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

### Usage

```python
import cornerstone260

mono = cornerstone260.Cornerstone260()
mono.connect('/dev/ttyS0', 9600)

# Enable handshake mode, so motion statements acknowledge when the move completes
mono.initialize()

print(mono.info())
print(mono.get_wavelength(), mono.get_units())
print(mono.get_grating())

# Move to 500 nm, then switch to grating 2
mono.set_wavelength(500.0)
mono.set_grating(2)

mono.close_shutter()
mono.disconnect()

# For a comprehensive list of classes and methods, use the help function
help(cornerstone260)

```

### Notes

The driver enables handshake mode in `initialize()`. Standard mode gives no completion
signal at all, so a `GOWAVE` returns its echo while the drive is still moving; the status
byte that handshake mode returns on completion is what makes `set_wavelength()` block
until the move has finished.

Accessories that are not fitted are reported rather than raised. A unit with no filter
wheel answers `0` to `FILTER?` and sets error 6, so `get_filter()` logs a warning and
returns `None`, and `get_limits()` omits the axis entirely.

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