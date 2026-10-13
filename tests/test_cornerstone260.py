# coding=utf-8
"""Tests for the Cornerstone 260 driver, run against a fake serial port."""

import pytest

from cornerstone260 import (
    Cornerstone260,
    CornerstoneError,
    OutputPort,
    ShutterState,
    Units,
)

# Responses observed from the real instrument, SN281 running firmware V03.10
REAL_RESPONSES = {
    "INFO?": "Oriel Instruments, Cornerstone 260, SN281,V03.10",
    "HANDSHAKE?": "0",
    "UNITS?": "nm",
    "WAVE?": "0.981",
    "GRAT?": "2,600,1000",
    "SHUTTER?": "O",
    "FILTER?": "0",
    "OUTPORT?": "2",
    "BANDPASS?": "0.000",
    "STEP?": "96231",
    "SLIT1MICRONS?": "0",
    "SLIT2MICRONS?": "0",
    "SLIT3MICRONS?": "0",
    "GRAT1LINES?": "1200",
    "GRAT1LABEL?": "350",
    "STB?": "00",
    "ERROR?": "",
}


class FakeSerial:
    """Stand-in for serial.Serial that replays the Cornerstone echo and handshake framing."""

    def __init__(self, responses=None, handshake=False, status="0"):
        self.responses = dict(REAL_RESPONSES if responses is None else responses)
        self.handshake = handshake
        self.status = status
        self.buffer = bytearray()
        self.written = []
        self.timeout = 1.0
        self.is_open = True

    def write(self, data):
        """Queue the echo, then the value line for a query, then the status byte."""
        statement = data.decode("ascii").strip()
        self.written.append(statement)
        self.buffer += data
        if statement.endswith("?"):
            self.buffer += self.responses.get(statement, "").encode("ascii") + b"\r\n"
        if self.handshake:
            self.buffer += self.status.encode("ascii") + b"\r\n"
        return len(data)

    def read_until(self, expected=b"\n"):
        """Pop bytes up to and including the terminator."""
        index = self.buffer.find(expected)
        if index == -1:
            chunk = bytes(self.buffer)
            self.buffer.clear()
            return chunk
        end = index + len(expected)
        chunk = bytes(self.buffer[:end])
        del self.buffer[:end]
        return chunk

    def read(self, size=1):
        """Pop up to size buffered bytes."""
        chunk = bytes(self.buffer[:size])
        del self.buffer[:size]
        return chunk

    def reset_input_buffer(self):
        """Discard buffered input."""
        self.buffer.clear()

    def flush(self):
        """Accept the flush the driver issues after every write."""

    def close(self):
        """Mark the port closed."""
        self.is_open = False


def make_device(**kwargs):
    """Return a connected driver backed by a fake serial port."""
    device = Cornerstone260(log=False)
    device.serial = FakeSerial(**kwargs)
    device._set_connected(True)  # pylint: disable=protected-access
    return device


def test_info_parses_the_format_the_firmware_actually_returns():
    """INFO? lacks the model number and field layout the manual documents."""
    info = make_device().info()
    assert info.vendor == "Oriel Instruments"
    assert info.model == "Cornerstone 260"
    assert info.serial_number == "281"
    assert info.firmware_version == "03.10"
    assert info.raw == REAL_RESPONSES["INFO?"]


def test_units_response_is_accepted_in_lower_case():
    """The manual promises upper case responses; the firmware answers "nm"."""
    assert make_device().get_units() is Units.NANOMETERS


def test_shutter_and_output_port_parse_into_enums():
    """Shutter and output port responses map onto their enums."""
    device = make_device()
    assert device.get_shutter() is ShutterState.OPEN
    assert device.get_output_port() is OutputPort.LATERAL


def test_grating_query_splits_into_number_density_and_label():
    """GRAT? returns three comma separated fields without spaces."""
    grating = make_device().get_grating()
    assert (grating.number, grating.lines_per_mm, grating.label) == (2, 600, "1000")


def test_wavelength_and_step_convert_to_numbers():
    """Wavelength is floating point and step position is an integer."""
    device = make_device()
    assert device.get_wavelength() == pytest.approx(0.981)
    assert device.get_step() == 96231


def test_echo_mismatch_raises_and_drains_the_buffer():
    """A desynchronised stream must not be parsed as if it were aligned."""
    device = make_device()
    device.serial.buffer += b"stale junk\n"
    with pytest.raises(CornerstoneError, match="Echo mismatch"):
        device.get_wavelength()
    assert device.serial.buffer == b""


def test_error_status_raises_with_the_decoded_message():
    """The handshake acknowledgement is decimal, so the error bit reads 32."""
    device = make_device(handshake=True, status="32",
                         responses={**REAL_RESPONSES, "ERROR?": "1"})
    device.handshake = True
    with pytest.raises(CornerstoneError, match="Command not understood"):
        device.get_wavelength()


def test_absent_accessory_warns_instead_of_raising():
    """Error 6 is a statement of fact about the hardware, not a failure to report."""
    device = make_device(handshake=True, status="32",
                         responses={**REAL_RESPONSES, "ERROR?": "6"})
    device.handshake = True
    assert device.get_filter() is None


def test_filter_absent_returns_none_rather_than_zero():
    """A zero filter position means no wheel, not position zero."""
    assert make_device().get_filter() is None


def test_slits_absent_return_none_rather_than_zero():
    """A zero slit width means the slit is absent or faulty."""
    device = make_device()
    assert [device.get_slit_microns(slit) for slit in (1, 2, 3)] == [None, None, None]


def test_limits_scale_with_groove_density():
    """Maximum wavelength is about 1600 nm at 1200 l/mm, so 600 l/mm reaches 3200 nm."""
    limits = make_device().get_limits()
    assert limits["wavelength_nm"] == pytest.approx((0.0, 3200.0))
    assert limits["grating"] == (1.0, 3.0)


def test_limits_omit_accessories_that_are_not_fitted():
    """Absent accessories get no entry rather than nominal limits."""
    limits = make_device().get_limits()
    assert "filter" not in limits
    assert not [key for key in limits if key.startswith("slit")]


def test_limits_include_accessories_that_are_fitted():
    """A fitted filter wheel and slit report their documented ranges."""
    device = make_device(responses={**REAL_RESPONSES, "FILTER?": "3", "SLIT1MICRONS?": "600"})
    limits = device.get_limits()
    assert limits["filter"] == (1.0, 6.0)
    assert limits["slit1"] == (6.0, 2000.0)


def test_motion_statements_are_formatted_as_the_manual_specifies():
    """Verify the statement text without a real instrument, since motion is never run here."""
    device = make_device()
    device.set_wavelength(500.0)
    device.set_grating(2)
    device.step(-25)
    device.close_shutter()
    assert device.serial.written == ["GOWAVE 500.0", "GRAT 2", "STEP -25", "SHUTTER C"]


def test_axis_dispatch_routes_to_the_matching_statement():
    """The generic motion interface reaches each named axis."""
    device = make_device()
    device.set_pos(500.0)
    device.set_pos(3, axis="grating")
    device.set_pos(600, axis="slit2")
    assert device.serial.written == ["GOWAVE 500.0", "GRAT 3", "SLIT2MICRONS 600"]


def test_unknown_axis_is_rejected():
    """An unrecognised axis name is a programming error, not a silent no-op."""
    device = make_device()
    with pytest.raises(ValueError, match="Unknown axis"):
        device.get_pos(axis="elevation")


@pytest.mark.parametrize("grating", [0, 4, -1])
def test_out_of_range_grating_is_rejected(grating):
    """The turret holds three gratings, so anything else is rejected locally."""
    with pytest.raises(ValueError, match="Grating must be one of"):
        make_device().set_grating(grating)


@pytest.mark.parametrize("position", [0, 7])
def test_out_of_range_filter_is_rejected(position):
    """Filter positions run 1 to 6."""
    with pytest.raises(ValueError, match="Filter position must be within"):
        make_device().set_filter(position)


def test_overlong_label_is_rejected_before_it_reaches_the_instrument():
    """Error 9 is avoidable, so the driver catches the length locally."""
    device = make_device()
    with pytest.raises(ValueError, match="at most 8 characters"):
        device.set_grating_label(1, "TOOLONGLABEL")
    assert device.serial.written == []


def test_unsupported_motion_methods_report_rather_than_pretend():
    """The base class methods with no Cornerstone equivalent send nothing."""
    device = make_device()
    assert not device.home()
    assert not device.is_homed()
    assert not device.close_loop()
    assert not device.is_loop_closed()
    assert device.serial.written == []


def test_statements_are_not_sent_when_disconnected():
    """Every statement checks the connection before touching the port."""
    device = Cornerstone260(log=False)
    assert device.get_wavelength() is None
    assert device.set_wavelength(500.0) is False


def test_empty_error_response_means_nothing_pending():
    """ERROR? answers with an empty line, not a zero, when no error is set."""
    assert make_device().get_error_code() is None


def test_status_byte_is_reported_in_hex():
    """STB? uses hex for the same byte the handshake reports in decimal."""
    device = make_device()
    assert device.get_status_byte() == "00"
    assert device.has_error() is False


def test_status_byte_error_value_is_detected():
    """Bit 5 set reads 20 in hex, the same byte the handshake reports as 32."""
    device = make_device(responses={**REAL_RESPONSES, "STB?": "20"})
    assert device.has_error() is True


def test_handshake_acknowledgement_of_zero_is_success():
    """The acknowledgement is a bare "0", not the "00" the manual's examples show."""
    device = make_device(handshake=True, status="0")
    device.handshake = True
    assert device.get_wavelength() == pytest.approx(0.981)


def test_error_status_without_a_code_is_reported_rather_than_mislabelled():
    """An error byte with no pending code must not be decoded as a known error."""
    device = make_device(handshake=True, status="32",
                         responses={**REAL_RESPONSES, "ERROR?": ""})
    device.handshake = True
    with pytest.raises(CornerstoneError, match="no error code"):
        device.get_wavelength()
