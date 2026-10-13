# coding=utf-8
"""
Driver for the Oriel Cornerstone 260 monochromator.

Statements and responses follow Appendix I of the Oriel Cornerstone 260 user's manual
(MCS260, Rev A).  Firmware V03.10 departs from that manual in a few places; each
departure is noted where the code accounts for it.

Connection is RS-232 at 9600 8N1, no flow control.  Statements are terminated with a
linefeed and responses with a carriage return and linefeed.  Every statement is echoed
back before its response, and in handshake mode an acknowledgement follows it.
"""

import enum
from dataclasses import dataclass
from typing import Dict, Optional, Tuple, Union

import serial

from hardware_device_base import HardwareMotionBase

DEFAULT_BAUD_RATE = 9600
DEFAULT_READ_TIMEOUT = 2.0
DEFAULT_MOVE_TIMEOUT = 90.0
RESYNC_SETTLE_SECONDS = 0.3

# The handshake acknowledgement is decimal, so the error bit reads 32, while STB? reports
# the same byte in hex, where it reads 20.  Manual 16.7 quotes both and explains neither.
HANDSHAKE_STATUS_OK = 0
STATUS_BYTE_OK = "00"

WAVELENGTH_AXIS = "wavelength"
GRATING_NUMBERS = (1, 2, 3)
SLIT_NUMBERS = (1, 2, 3)
FILTER_RANGE = (1, 6)
SLIT_RANGE_MICRONS = (6, 2000)
LABEL_MAX_LENGTH = 8

# Maximum wavelength scales inversely with groove density, about 1600 nm at 1200 l/mm
REFERENCE_LINES_PER_MM = 1200
REFERENCE_MAX_WAVELENGTH_NM = 1600.0

ERROR_MESSAGES = {
    "0": "System error (miscellaneous)",
    "1": "Command not understood",
    "2": "Bad parameter used in command",
    "3": "Destination position for wavelength motion not allowed",
    "6": "Accessory not present",
    "7": "Accessory already in specified position",
    "8": "Could not home wavelength drive",
    "9": "Label too long",
}
ACCESSORY_ABSENT_CODE = "6"


class CornerstoneError(RuntimeError):
    """Raised when the monochromator reports an error or breaks the reply framing."""


class Units(str, enum.Enum):
    """Wavelength units that motion statements and queries are expressed in."""

    NANOMETERS = "NM"
    MICROMETERS = "UM"
    WAVENUMBERS = "WN"


class ShutterState(str, enum.Enum):
    """Position of the shutter at the input port."""

    OPEN = "O"
    CLOSED = "C"


class OutputPort(str, enum.Enum):
    """Output port selected by the motorized flip mirror."""

    AXIAL = "1"
    LATERAL = "2"


@dataclass(frozen=True)
class GratingInfo:
    """Grating number, groove density and user label, as reported by GRAT?."""

    number: int
    lines_per_mm: int
    label: str


@dataclass(frozen=True)
class InstrumentInfo:
    """Instrument identity, as reported by INFO?."""

    vendor: str
    model: str
    serial_number: str
    firmware_version: str
    raw: str


def _looks_like_version(field: str) -> bool:
    """Return True for an INFO? field shaped like a firmware version, e.g. V03.10."""
    return len(field) > 1 and field[0].upper() == "V" and field[1].isdigit()


class Cornerstone260(HardwareMotionBase):
    """Controller class for the Oriel Cornerstone 260 monochromator."""

    # The instrument exposes a broad flat command set, so a wide public surface is expected.
    # pylint: disable=too-many-public-methods

    def __init__(self, log: bool = True, logfile: str = __name__.rsplit(".", 1)[-1],
                 read_timeout: float = DEFAULT_READ_TIMEOUT,
                 move_timeout: float = DEFAULT_MOVE_TIMEOUT) -> None:
        """
        Class to handle communications with the monochromator.

        :param log: Boolean, whether to log to file or not
        :param logfile: Filename for log
        :param read_timeout: Float, seconds to wait for an echo or a query response
        :param move_timeout: Float, seconds to wait for a motion statement to acknowledge

        NOTE: default is INFO level logging, use set_verbose to increase verbosity.
        """
        super().__init__(log, logfile)

        self.serial: Optional[serial.Serial] = None
        self.read_timeout = read_timeout
        self.move_timeout = move_timeout
        self.handshake = False

    # connection handling
    def connect(self, host, port=DEFAULT_BAUD_RATE, con_type: str = "serial") -> None:
        """
        Open the serial connection to the monochromator.

        :param host: String, serial device path, e.g. "/dev/ttyS0"
        :param port: Int, baud rate, fixed at 9600 on this instrument
        :param con_type: String, only "serial" is supported
        """
        # pylint: disable=W0221
        if not self.validate_connection_params((host, port)):
            self.report_error(f"Invalid connection parameters: {host}:{port}")
            self._set_connected(False)
            return
        if con_type != "serial":
            self.report_error("con_type must be 'serial'")
            self._set_connected(False)
            return
        try:
            self.serial = serial.Serial(
                port=host,
                baudrate=port,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.read_timeout,
                exclusive=True,
            )
        except (serial.SerialException, OSError) as ex:
            self._set_connected(False)
            self.report_error(f"Failed to open {host}: {ex}")
            raise IOError(f"Failed to open {host}: {ex}") from ex

        self._set_connected(True)
        self.handshake = False
        self.report_info(f"Serial connection opened: {host} at {port} baud")

    def disconnect(self) -> None:
        """Close the serial connection."""
        if not self.is_connected():
            self.report_warning("Already disconnected from device")
            return
        try:
            self.serial.close()
        except (serial.SerialException, OSError) as ex:
            self.report_error(f"Disconnection error: {ex}")
        finally:
            self.serial = None
            self._set_connected(False)
            self.report_info("Serial connection closed")

    def initialize(self) -> bool:
        """Enable handshake mode so statements acknowledge on completion."""
        if not self.is_connected():
            self.report_error("Device is not connected")
            return False
        self.set_handshake(True)
        self.initialized = self.get_handshake() is True
        if not self.initialized:
            self.report_error("Failed to enable handshake mode")
        return self.initialized

    # transport primitives
    def _send_command(self, command: str) -> bool:  # pylint: disable=W0221
        """
        Write one statement to the instrument.

        :param command: String, statement to send without its terminator
        :return: True if the statement was written
        """
        if not self.is_connected():
            self.report_error("Device is not connected")
            return False
        self.serial.write(command.encode("ascii") + b"\n")
        self.serial.flush()
        self.report_debug(f"Sent: {command}")
        return True

    def _read_reply(self, timeout: Optional[float] = None) -> Optional[str]:  # pylint: disable=W0221
        """
        Read one terminated line and strip its terminator.

        :param timeout: Float, seconds to wait, defaulting to the read timeout
        :return: The line, or None if the device is not connected
        """
        if not self.is_connected():
            self.report_error("Device is not connected")
            return None
        if timeout is not None:
            self.serial.timeout = timeout
        try:
            raw = self.serial.read_until(b"\n")
        finally:
            self.serial.timeout = self.read_timeout
        if not raw.endswith(b"\n"):
            raise CornerstoneError(f"Timed out reading reply, received {bytes(raw)!r}")
        reply = raw.decode("ascii", errors="replace").strip()
        self.report_debug(f"Received: {reply}")
        return reply

    def _resynchronise(self) -> None:
        """Drain buffered input so the next exchange starts on a line boundary."""
        self.serial.timeout = RESYNC_SETTLE_SECONDS
        try:
            while self.serial.read(256):
                pass
        finally:
            self.serial.timeout = self.read_timeout
        self.serial.reset_input_buffer()

    def _exchange(self, statement: str, expect_value: bool = False,
                  timeout: Optional[float] = None) -> Optional[str]:
        """
        Send one statement and read the reply it frames.

        :param statement: String, statement to send
        :param expect_value: Boolean, True for a query, which returns a value line
        :param timeout: Float, seconds to allow for the value and status lines
        :return: The value line for a query, otherwise None
        """
        # The lock spans the whole exchange rather than just the write: a reply is made of
        # an echo line plus optional value and status lines, and a second caller writing
        # between those reads would consume another thread's lines.
        with self.lock:
            return self._exchange_locked(statement, expect_value, timeout)

    def _exchange_locked(self, statement: str, expect_value: bool,
                         timeout: Optional[float]) -> Optional[str]:
        """Perform an exchange with the lock already held."""
        if not self._send_command(statement):
            return None
        echo = self._read_reply()
        if echo != statement:
            self._resynchronise()
            raise CornerstoneError(f"Echo mismatch: sent {statement!r}, received {echo!r}")
        value = self._read_reply(timeout) if expect_value else None
        if self.handshake:
            self._check_status(self._read_reply(timeout))
        return value

    def _check_status(self, status: Optional[str]) -> None:
        """Inspect the handshake acknowledgement and raise on anything but success."""
        if status is None:
            return
        try:
            reported = int(status)
        except ValueError as ex:
            raise CornerstoneError(f"Unparseable status byte {status!r}") from ex
        if reported == HANDSHAKE_STATUS_OK:
            return
        code = self._fetch_error_code()
        if not code:
            raise CornerstoneError(f"Status {status} reported an error, but no error code")
        message = ERROR_MESSAGES.get(code, f"Unknown error code {code!r}")
        if code == ACCESSORY_ABSENT_CODE:
            self.report_warning(f"{message} (status {status})")
            return
        raise CornerstoneError(f"{message} (status {status}, error {code})")

    def _fetch_error_code(self) -> str:
        """Read ERROR? with the lock held, without recursing into the status check."""
        if not self._send_command("ERROR?"):
            return ""
        self._read_reply()
        # ERROR? answers with an empty line, not a zero, when nothing is pending
        code = self._read_reply()
        if self.handshake:
            self._read_reply()
        return code or ""

    def _query(self, statement: str) -> Optional[str]:
        """Send a query and return its value line."""
        return self._exchange(statement, expect_value=True)

    def _command(self, statement: str, timeout: Optional[float] = None) -> bool:
        """Send a command and wait for it to acknowledge."""
        if not self.is_connected():
            self.report_error("Device is not connected")
            return False
        self._exchange(statement, expect_value=False,
                       timeout=self.move_timeout if timeout is None else timeout)
        return True

    # parameter validation
    @staticmethod
    def _valid_grating(grating: int) -> int:
        """Return the grating number, rejecting anything the turret cannot hold."""
        if grating not in GRATING_NUMBERS:
            raise ValueError(f"Grating must be one of {GRATING_NUMBERS}, got {grating}")
        return grating

    @staticmethod
    def _valid_slit(slit: int) -> int:
        """Return the slit number, rejecting anything the instrument does not have."""
        if slit not in SLIT_NUMBERS:
            raise ValueError(f"Slit must be one of {SLIT_NUMBERS}, got {slit}")
        return slit

    @staticmethod
    def _valid_filter(position: int) -> int:
        """Return the filter position, rejecting anything outside the wheel's range."""
        if not FILTER_RANGE[0] <= position <= FILTER_RANGE[1]:
            raise ValueError(f"Filter position must be within {FILTER_RANGE}, got {position}")
        return position

    @staticmethod
    def _valid_label(label: str) -> str:
        """Return the label, rejecting anything the instrument would truncate."""
        if len(label) > LABEL_MAX_LENGTH:
            raise ValueError(f"Label must be at most {LABEL_MAX_LENGTH} characters: {label!r}")
        return label

    # wavelength
    def get_wavelength(self) -> Optional[float]:
        """Return the current wavelength in the units currently selected."""
        reply = self._query("WAVE?")
        return float(reply) if reply else None

    def set_wavelength(self, wavelength: float) -> bool:
        """Move the wavelength drive to the given wavelength in the current units."""
        return self._command(f"GOWAVE {wavelength}")

    def get_step(self) -> Optional[int]:
        """Return the step position measured from the physical home sensors."""
        reply = self._query("STEP?")
        return int(reply) if reply else None

    def step(self, steps: int) -> bool:
        """Move the wavelength drive by a signed number of steps."""
        return self._command(f"STEP {int(steps)}")

    def abort(self) -> bool:
        """Stop wavelength motion immediately."""
        return self._command("ABORT", timeout=self.read_timeout)

    def calibrate(self, wavelength: float) -> bool:
        """Define the current position as the given wavelength and store the offset."""
        return self._command(f"CALIBRATE {wavelength}")

    def get_units(self) -> Optional[Units]:
        """Return the units that motion statements and queries are expressed in."""
        reply = self._query("UNITS?")
        # Manual 16.2 says responses are upper case; firmware V03.10 answers "nm"
        return Units(reply.upper()) if reply else None

    def set_units(self, units: Units) -> bool:
        """Set the units that motion statements and queries are expressed in."""
        return self._command(f"UNITS {Units(units).value}", timeout=self.read_timeout)

    # gratings
    def get_grating(self) -> Optional[GratingInfo]:
        """Return the selected grating's number, groove density and label."""
        reply = self._query("GRAT?")
        if not reply:
            return None
        number, lines, label = reply.split(",", 2)
        return GratingInfo(number=int(number), lines_per_mm=int(lines), label=label)

    def set_grating(self, grating: int) -> bool:
        """Move the turret to the given grating."""
        return self._command(f"GRAT {self._valid_grating(grating)}")

    def get_grating_lines(self, grating: int) -> Optional[int]:
        """Return the groove density of the given grating in lines per millimetre."""
        reply = self._query(f"GRAT{self._valid_grating(grating)}LINES?")
        return int(reply) if reply else None

    def set_grating_lines(self, grating: int, lines_per_mm: int) -> bool:
        """Set the groove density used to calculate wavelength motion."""
        return self._command(f"GRAT{self._valid_grating(grating)}LINES {int(lines_per_mm)}",
                             timeout=self.read_timeout)

    def get_grating_label(self, grating: int) -> Optional[str]:
        """Return the user label of the given grating."""
        return self._query(f"GRAT{self._valid_grating(grating)}LABEL?")

    def set_grating_label(self, grating: int, label: str) -> bool:
        """Set the user label of the given grating."""
        return self._command(
            f"GRAT{self._valid_grating(grating)}LABEL {self._valid_label(label)}",
            timeout=self.read_timeout)

    def get_grating_factor(self, grating: int) -> Optional[float]:
        """Return the calibration factor of the given grating."""
        reply = self._query(f"GRAT{self._valid_grating(grating)}FACTOR?")
        return float(reply) if reply else None

    def set_grating_factor(self, grating: int, factor: float) -> bool:
        """Set the calibration factor used to calculate wavelength motion."""
        return self._command(f"GRAT{self._valid_grating(grating)}FACTOR {factor}",
                             timeout=self.read_timeout)

    def get_grating_offset(self, grating: int) -> Optional[float]:
        """Return the calibration offset of the given grating."""
        reply = self._query(f"GRAT{self._valid_grating(grating)}OFFSET?")
        return float(reply) if reply else None

    def set_grating_offset(self, grating: int, offset: float) -> bool:
        """Set the calibration offset used to calculate wavelength motion."""
        return self._command(f"GRAT{self._valid_grating(grating)}OFFSET {offset}",
                             timeout=self.read_timeout)

    def get_grating_zero(self, grating: int) -> Optional[float]:
        """Return the zero parameter of the given grating."""
        reply = self._query(f"GRAT{self._valid_grating(grating)}ZERO?")
        return float(reply) if reply else None

    def set_grating_zero(self, grating: int, zero: float) -> bool:
        """Set the zero parameter, which depends on the grating mount configuration."""
        return self._command(f"GRAT{self._valid_grating(grating)}ZERO {zero}",
                             timeout=self.read_timeout)

    # accessories
    def get_shutter(self) -> Optional[ShutterState]:
        """Return the shutter position."""
        reply = self._query("SHUTTER?")
        return ShutterState(reply.upper()) if reply else None

    def set_shutter(self, state: ShutterState) -> bool:
        """Open or close the shutter."""
        return self._command(f"SHUTTER {ShutterState(state).value}",
                             timeout=self.read_timeout)

    def open_shutter(self) -> bool:
        """Open the shutter."""
        return self.set_shutter(ShutterState.OPEN)

    def close_shutter(self) -> bool:
        """Close the shutter."""
        return self.set_shutter(ShutterState.CLOSED)

    def get_filter(self) -> Optional[int]:
        """
        Return the filter wheel position.

        A unit with no wheel fitted, or a wheel out of position, answers 0 and sets
        error 6 in the status byte, which is reported as a warning rather than raised.
        """
        reply = self._query("FILTER?")
        if not reply:
            return None
        position = int(reply)
        if position == 0:
            self.report_warning("No filter wheel present, or wheel out of position")
            return None
        return position

    def set_filter(self, position: int) -> bool:
        """Move the filter wheel to the given position."""
        return self._command(f"FILTER {self._valid_filter(position)}")

    def get_filter_label(self, position: int) -> Optional[str]:
        """Return the label of the given filter position."""
        return self._query(f"FILTER{self._valid_filter(position)}LABEL?")

    def set_filter_label(self, position: int, label: str) -> bool:
        """Set the label of the given filter position."""
        return self._command(
            f"FILTER{self._valid_filter(position)}LABEL {self._valid_label(label)}",
            timeout=self.read_timeout)

    def get_slit_microns(self, slit: int) -> Optional[int]:
        """Return the width of the given motorized slit, or None if it is not fitted."""
        reply = self._query(f"SLIT{self._valid_slit(slit)}MICRONS?")
        if not reply:
            return None
        microns = int(reply)
        if microns == 0:
            self.report_warning(f"Slit {slit} not present or faulty")
            return None
        return microns

    def set_slit_microns(self, slit: int, microns: int) -> bool:
        """
        Open the given motorized slit to the requested width.

        The instrument clamps to its own 6 to 2000 micron range and moves in 6 micron
        steps, so the requested width is passed through unaltered.
        """
        return self._command(f"SLIT{self._valid_slit(slit)}MICRONS {int(microns)}")

    def get_bandpass(self) -> Optional[float]:
        """Return the bandpass the slits are automatically held at, or 0 when disabled."""
        reply = self._query("BANDPASS?")
        return float(reply) if reply else None

    def set_bandpass(self, bandpass: float) -> bool:
        """Set the bandpass the slits track automatically; 0 disables the adjustment."""
        return self._command(f"BANDPASS {bandpass}")

    def get_output_port(self) -> Optional[OutputPort]:
        """Return the output port the flip mirror is directing light to."""
        reply = self._query("OUTPORT?")
        return OutputPort(reply) if reply else None

    def set_output_port(self, port: OutputPort) -> bool:
        """Move the flip mirror to select the axial or lateral output port."""
        return self._command(f"OUTPORT {OutputPort(port).value}")

    # system
    def info(self) -> Optional[InstrumentInfo]:
        """
        Return the instrument identity.

        Firmware V03.10 answers "Oriel Instruments, Cornerstone 260, SN281,V03.10", which
        has neither the model number nor the field layout the manual documents, so the
        serial number and version are located by prefix and the raw string is preserved.
        """
        reply = self._query("INFO?")
        if not reply:
            return None
        fields = [field.strip() for field in reply.split(",")]
        return InstrumentInfo(
            vendor=fields[0],
            model=fields[1] if len(fields) > 1 else "",
            serial_number=next((f[2:] for f in fields if f.upper().startswith("SN")), ""),
            firmware_version=next((f[1:] for f in fields if _looks_like_version(f)), ""),
            raw=reply,
        )

    def get_handshake(self) -> Optional[bool]:
        """Return True when the instrument acknowledges every statement."""
        reply = self._query("HANDSHAKE?")
        return reply == "1" if reply else None

    def set_handshake(self, enabled: bool) -> bool:
        """Turn completion acknowledgement on or off."""
        with self.lock:
            if not self._send_command(f"HANDSHAKE {1 if enabled else 0}"):
                return False
            self._read_reply()
            # The mode changes as this statement is processed, so HANDSHAKE 1 is itself
            # acknowledged but HANDSHAKE 0 is not; drain rather than encode the asymmetry.
            self._resynchronise()
            self.handshake = enabled
        return True

    def get_error_code(self) -> Optional[str]:
        """Return the pending error code, clearing it, or None when nothing is pending."""
        return self._query("ERROR?") or None

    def get_status_byte(self) -> Optional[str]:
        """Return the status byte in hex, clearing it; "00" is no error and "20" is an error."""
        return self._query("STB?")

    def has_error(self) -> bool:
        """Return True when the status byte reports a pending error, clearing it."""
        return self.get_status_byte() not in (STATUS_BYTE_OK, None)

    # HardwareMotionBase interface
    def get_pos(self, axis: str = WAVELENGTH_AXIS) -> Union[float, int, None]:  # pylint: disable=W0221
        """
        Return the position of the named axis.

        :param axis: String, one of "wavelength", "grating", "filter", "step" or "slitN"
        """
        if axis == WAVELENGTH_AXIS:
            return self.get_wavelength()
        if axis == "grating":
            grating = self.get_grating()
            return grating.number if grating else None
        if axis == "filter":
            return self.get_filter()
        if axis == "step":
            return self.get_step()
        if axis.startswith("slit"):
            return self.get_slit_microns(int(axis[len("slit"):]))
        raise ValueError(f"Unknown axis {axis!r}")

    def set_pos(self, position: Union[float, int], axis: str = WAVELENGTH_AXIS) -> bool:
        """
        Move the named axis to the given position.

        :param position: Float or int, target in the current units for wavelength
        :param axis: String, one of "wavelength", "grating", "filter", "step" or "slitN"
        """
        # pylint: disable=W0221
        if axis == WAVELENGTH_AXIS:
            return self.set_wavelength(float(position))
        if axis == "grating":
            return self.set_grating(int(position))
        if axis == "filter":
            return self.set_filter(int(position))
        if axis == "step":
            return self.step(int(position))
        if axis.startswith("slit"):
            return self.set_slit_microns(int(axis[len("slit"):]), int(position))
        raise ValueError(f"Unknown axis {axis!r}")

    def get_limits(self) -> Optional[Dict[str, Tuple[float, float]]]:
        """
        Return the usable range of each axis present on this instrument.

        The wavelength maximum depends on the installed grating, so it is derived from
        the selected grating's groove density rather than hard coded.  It is reported in
        nanometres whatever the UNITS setting, because the wavenumber axis is reciprocal
        and does not map onto an interval that shares the other axes' sense.

        Accessories that are absent are omitted rather than reported with nominal limits.
        """
        grating = self.get_grating()
        if grating is None:
            return None
        max_wavelength = (REFERENCE_MAX_WAVELENGTH_NM
                          * REFERENCE_LINES_PER_MM / grating.lines_per_mm)
        limits: Dict[str, Tuple[float, float]] = {
            "wavelength_nm": (0.0, max_wavelength),
            "grating": (float(GRATING_NUMBERS[0]), float(GRATING_NUMBERS[-1])),
        }
        if self.get_filter() is not None:
            limits["filter"] = (float(FILTER_RANGE[0]), float(FILTER_RANGE[1]))
        for slit in SLIT_NUMBERS:
            if self.get_slit_microns(slit) is not None:
                limits[f"slit{slit}"] = (float(SLIT_RANGE_MICRONS[0]),
                                         float(SLIT_RANGE_MICRONS[1]))
        return limits

    def home(self) -> bool:
        """Not supported: the wavelength drive homes itself at power on."""
        self.report_warning("home() is not supported; the drive homes itself at power on")
        return False

    def is_homed(self) -> bool:
        """Not supported: the instrument exposes no homing state to query."""
        self.report_warning("is_homed() is not supported; no homing state is reported")
        return False

    def close_loop(self) -> bool:
        """Not supported: the instrument has no externally controllable servo loop."""
        self.report_warning("close_loop() is not supported; the loop is internal")
        return False

    def is_loop_closed(self) -> bool:
        """Not supported: the instrument exposes no loop state to query."""
        self.report_warning("is_loop_closed() is not supported; no loop state is reported")
        return False
