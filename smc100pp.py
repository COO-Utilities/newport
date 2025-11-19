# coding=utf-8
"""
The following stage controller commands are available. Note that many
are not implemented at the moment.  The asterisk indicates the commands
that are implemented.

AC    Set/Get acceleration
BA    Set/Get backlash compensation
BH    Set/Get hysteresis compensation
DV    Set/Get driver voltage Not for PP
FD    Set/Get low pass filter for Kd Not for PP
FE    Set/Get following error limit Not for PP
FF    Set/Get friction compensation Not for PP
FR    Set/Get stepper motor configuration Not for CC
HT    Set/Get HOME search type
ID    Set/Get stage identifier
JD    Leave JOGGING state
JM    Enable/disable keypad
JR    Set/Get jerk time
KD    Set/Get derivative gain Not for PP
KI    Set/Get integral gain Not for PP
KP    Set/Get proportional gain Not for PP
KV    Set/Get velocity feed forward Not for PP
MM    Enter/Leave DISABLE state
OH    Set/Get HOME search velocity
OR  * Execute HOME search
OT    Set/Get HOME search time-out
PA  * Move absolute
PR  * Move relative
PT    Get motion time for a relative move
PW    Enter/Leave CONFIGURATION state
QI    Set/Get motor’s current limits
RA    Get analog input value
RB    Get TTL input value
RS  * Reset controller
SA    Set/Get controller’s RS-485 address
SB    Set/Get TTL output value
SC    Set/Get control loop state Not for PP
SE    Configure/Execute simultaneous started move
SL  * Set/Get negative software limit
SR  * Set/Get positive software limit
ST    Stop motion
SU    Set/Get encoder increment value Not for PP
TB    Get command error string
TE  * Get last command error
TH    Get set-point position
TP  * Get current position
TS  * Get positioner error and controller state
VA    Set/Get velocity
VB    Set/Get base velocity Not for CC
VE    Get controller revision information
ZT  * Get all axis parameters
ZX    Set/Get SmartStage configuration

The values below as of 2025-May-19

For stage 1 & 2 current values are:
80 - Acceleration
-3600 - negative software limit, from 1SL?
+3600 - positive software limit, from 1SR?
20 - Microstep factor
0.0200682 - Full step value
0.04 - Jerk time in seconds
8 - deg/s Home velocity
1980 - Home timeout in seconds
0.3 - Peak current limit in Amperes
2 - Controller's RS485 address
8 - deg/s Move velocity
0 - deg/s Base velocity
ESP stage check enabled
Home type: use MZ switch only
Backlash and hysteresis compensations are disabled.
"""

import errno
import time
import socket
from hardware_device_base.hardware_motion_base import HardwareMotionBase


class StageController(HardwareMotionBase):
    """
    Controller class for Newport SMC100PP Stage Controller.
    """
    # pylint: disable=too-many-instance-attributes

    controller_commands = ["OR",    # Execute HOME search
                           "PA",    # Absolute move
                           "PR",    # Move relative
                           "RS",    # Reset controller
                           "SL",    # Set/Get positive software limit
                           "SR",    # Set/Get negative software limit
                           "TE",    # Get last command error
                           "TP",    # Get current position
                           "TS",    # Get positioner error and controller state
                           "ZT"     # Get all axis parameters
                           ]
    return_value_commands = ["TE", "TP", "TS"]
    parameter_commands = ["PA", "PR", "SL", "SR"]
    end_code_list = ['32', '33', '34', '35']
    not_ref_list = ['0A', '0B', '0C', '0D', '0F', '10', '11']
    moving_list = ['28']
    msg = {
        "0A": "NOT REFERENCED from reset.",
        "0B": "NOT REFERENCED from HOMING.",
        "0C": "NOT REFERENCED from CONFIGURATION.",
        "0D": "NOT REFERENCED from DISABLE.",
        "0E": "NOT REFERENCED from READY.",
        "0F": "NOT REFERENCED from MOVING.",
        "10": "NOT REFERENCED ESP stage error.",
        "11": "NOT REFERENCED from JOGGING.",
        "14": "CONFIGURATION.",
        "1E": "HOMING commanded from RS-232-C.",
        "1F": "HOMING commanded by SMC-RC.",
        "28": "MOVING.",
        "32": "READY from HOMING.",
        "33": "READY from MOVING.",
        "34": "READY from DISABLE.",
        "35": "READY from JOGGING.",
        "3C": "DISABLE from READY.",
        "3D": "DISABLE from MOVING.",
        "3E": "DISABLE from JOGGING.",
        "46": "JOGGING from READY.",
        "47": "JOGGING from DISABLE."
    }
    error = {
        "@": "No error.",
        "A": "Unknown message code or floating point controller address.",
        "B": "Controller address not correct",
        "C": "Parameter missing or out of range.",
        "D": "Command not allowed.",
        "E": "Home sequence already started.",
        "F": "ESP stage name unknown.",
        "G": "Displacement out of limits.",
        "H": "Command not allowed in NOT REFERENCED state.",
        "I": "Command not allowed in CONFIGURATION state.",
        "J": "Command not allowed in DISABLE state.",
        "K": "Command not allowed in READY state.",
        "L": "Command not allowed in HOMING state.",
        "M": "Command not allowed in MOVING state.",
        "N": "Current position out of software limit.",
        "S": "Communication Time Out.",
        "U": "Error during EEPROM access.",
        "V": "Error durring command execution.",
        "W": "Command not allowed for PP version.",
        "X": "Command not allowed for CC version."
    }
    last_error = ""

    def __init__(self, num_stages=2, move_rate=5.0,
                 log: bool =True, logfile: str =__name__.rsplit(".", 1)[-1]):

        """
        Class to handle communications with the stage controller and any faults

        :param num_stages: Int, number of stages daisey-chained
        :param move_rate: Float, move rate in degrees per second
        :param log: Boolean, whether to log to file or not
        :param logfile: Filename for log

        NOTE: default is INFO level logging, use set_verbose to increase verbosity.
        """

        # thread lock
        super().__init__(log, logfile)

        # Set up socket
        self.socket = None
        self.connected = False

        # number of daisy-chained stages
        self.num_stages = num_stages

        # stage rate in degrees per second
        self.move_rate = move_rate

        # current values
        self.current_position = [0.0] * (num_stages + 1)
        self.current_limits = [(0., 0.)] * (num_stages + 1)

    def connect(self, host, port) -> None: # pylint: disable=W0221
        """ Connect to stage controller. """
        if self.validate_connection_params((host, port)):
            if self.socket is None:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.socket.connect((host, port))
                self.logger.info("Connected to %(host)s:%(port)s", {
                    'host': host,
                    'port': port
                })
                self._set_status((0, f"Connected to {host}:{port}"))
                self._set_connected(True)

            except OSError as ex:
                if ex.errno == errno.EISCONN:
                    self.logger.info("Already connected")
                    self._set_status((0, "Already connected"))
                    self._set_connected(True)
                else:
                    self.logger.error("Connection error: %s", ex.strerror)
                    self._set_status((-1, f"Connection error {ex.strerror}"))
                    self._set_connected(False)

            # clear socket
            if self.connected:
                self._clear_socket()
        else:
            self.logger.error("Invalid connection arguments: %s", (host, port))
            self._set_status((-1, f"Invalid connection arguments: {host}:{port}"))

    def disconnect(self):
        """ Disconnect stage controller. """
        if not self.is_connected():
            self.logger.warning("Already disconnected from device")
            self._set_status((0, "Already disconnected from device"))
            return
        try:
            self.logger.info("Disconnecting from device")
            if self.socket:
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
                self.socket = None
            self._set_connected(False)
            self.logger.info("Disconnected controller")
            self._set_status((0, "Disconnected controller"))

        except OSError as ex:
            self.logger.error("Disconnection error: %s", ex.strerror)
            self._set_connected(False)
            self.socket = None

    def _clear_socket(self):
        """ Clear socket buffer. """
        if self.socket is not None:
            self.socket.setblocking(False)
            while True:
                try:
                    _ = self.socket.recv(1024)
                except BlockingIOError:
                    break
            self.socket.setblocking(True)

    def _read_reply(self):
        """ Read return value from controller """
        # Return value commands

        # Get return value
        recv = self.socket.recv(2048)
        recv_len = len(recv)
        if self.logger:
            self.logger.debug("Return: len = %d, Value = %s", recv_len, recv)

        # Are we a valid return value?
        if recv_len in [6, 11, 12, 13, 14]:
            if self.logger:
                self.logger.debug("Return value validated")
        return str(recv.decode('utf-8'))

    def _read_params(self):
        """ Read stage controller parameters """
        # Get return value
        recv = self.socket.recv(2048)

        # Did we get all the params?
        tries = 5
        while tries > 0 and b'PW0' not in recv:
            recv += self.socket.recv(2048)
            tries -= 1

        if b'PW0' in recv:
            recv_len = len(recv)
            if self.logger:
                self.logger.debug("ZT Return: len = %d", recv_len)
        else:
            if self.logger:
                self.logger.warning("ZT command timed out")

        return str(recv.decode('utf-8'))

    def _read_blocking(self, stage_id=1, timeout=15) -> bool:
        """ Block while reading from the controller.
        :param stage_id: Int, stage id
        :param timeout: Timeout for blocking read
        """

        start = time.time()

        # Non-return value commands eventually return state output
        sleep_time = 0.1
        start_time = time.time()
        print_it = 0
        recv = None
        while time.time() - start_time < timeout:
            # Check state
            statecmd = f'{stage_id}TS\r\n'
            statecmd = statecmd.encode('utf-8')
            self.socket.send(statecmd)
            time.sleep(sleep_time)
            recv = self.socket.recv(1024)

            # Valid state return
            if len(recv) == 11:
                # Parse state
                recv = recv.rstrip()
                code = str(recv[-2:].decode('utf-8'))

                # Valid end code or not referenced code (done)
                if code in self.end_code_list or code in self.not_ref_list:
                    return True

                if print_it >= 10:
                    msg = (f"{time.time()-start:05.2f} "
                           f"{self.msg.get(code, 'Unknown state'):s}")
                    self.logger.info(msg)
                    print_it = 0

            # Invalid state return (done)
            else:
                self.logger.warning("Bad %dTS return: %s", stage_id, recv)
                self._set_status((-1, f"Bad {stage_id}TS return: {recv}"))
                return False

            # Increment tries and read state again
            print_it += 1

        # If we get here, we ran out of tries
        recv = recv.rstrip()
        code = str(recv[-2:].decode('utf-8'))
        self.logger.warning("Command timed out, final state: %s",
                            self.msg.get(code, "Unknown state"))
        return False

    def _send_serial_command(self, stage_id=1, command='') -> bool:
        """
        Send serial command to stage controller

        :param stage_id: Int, stage position in the daisy chain starting with 1
        :param command: String, command to send to stage controller
        :return: Bool, True if successful, False otherwise
        """
        # Prep command
        cmd_send = f"{stage_id}{command}\r\n"
        self.logger.debug("Sending command:%s", cmd_send)
        cmd_encoded = cmd_send.encode('utf-8')

        # check connection
        if self.is_connected():
            try:
                self.socket.settimeout(30)
                # Send command
                self.socket.send(cmd_encoded)
                time.sleep(.05)

            except socket.error as ex:
                self.logger.error("Socket error: %s", ex)
                self._set_status((-1, f"Socket error: {ex}"))
                return False
        else:
            self.logger.error("Not connected to stage controller")
            self._set_status((-1, "Not connected to stage controller"))
            return False

        return True

    def _send_command(self, command="", parameters=None, stage_id=1, custom_command=False):
        """
        Send a command to the stage controller

        :param command: String, command to send to the stage controller
        :param parameters: List of string parameters associated with cmd
        :param stage_id: Int, stage position in the daisy chain starting with 1
        :param custom_command: Boolean, if true, command is custom
        :return:
        """
        # pylint: disable=W0221
        # verify cmd and stage_id
        if self._verify_send_command(command, stage_id, custom_command):

            # Check if the command should have parameters
            if command in self.parameter_commands and parameters:
                self.logger.debug("Adding parameters")
                parameters = [str(x) for x in parameters]
                parameters = " ".join(parameters)
                command += parameters

            self.logger.debug("Input command: %s", command)

            # Send serial command
            with self.lock:
                result = self._send_serial_command(stage_id, command)
        else:
            self.logger.error("Command not valid: %s", command)
            self._set_status((-1, f"Command not valid: {command}"))
            result = False

        return result

    def _verify_send_command(self, cmd, stage_id, custom_command=False) -> bool:
        """ Verify cmd and stage_id

        :param cmd: String, command to send to the stage controller
        :param stage_id: Int, stage position in the daisy chain starting with 1
        :param custom_command: Boolean, if true, command is custom
        :return: dictionary {'elaptime': time, 'data|error': string_message}"""

        # return value
        ret = False
        # Do we have a connection?
        if not self.connected:
            self.logger.warning("Not connected to controller")
            self._set_status((-1, "Not connected to controller"))

        # Is stage id valid?
        elif not self._verify_stage_id(stage_id):
            self.logger.warning("Not a valid stage id: %d", stage_id)
            self._set_status((-1, f"Not a valid stage id: {stage_id}"))

        else:
            # Do we have a legal command?
            if cmd.rstrip().upper() in self.controller_commands:
                self.logger.debug("Command validated: %s", cmd)
                ret = True
            else:
                if not custom_command:
                    self.logger.error("Command invalid: %s", cmd)
                    self._set_status((-1, f"Command invalid: {cmd}"))
                else:
                    self.logger.debug("Command is custom: %s", cmd)
                    self._set_status((0, f"{cmd} is a custom command"))
                    ret = True

        return ret

    def _verify_stage_id(self, stage_id):
        """ Check that the stage id is legal

        :param stage_id: Int, stage position in the daisy chain starting with 1
        :return: True if stage id is legal
        """
        if stage_id > self.num_stages or stage_id < 1:
            is_valid = False
        else:
            is_valid = True

        return is_valid

    def _verify_move_state(self, stage_id, position, move_type='absolute') -> bool:
        """ Verify that the move is allowed
        :param stage_id: Int, stage position in the daisy chain starting with 1
        :param position: String, move position
        :param move_type: String, move type: 'absolute' or 'relative'
        :return: True if move is allowed"""

        # Return value
        ret = False
        # Verify inputs
        if position is None or stage_id is None:
            self.logger.error("Must specify both position and stage_id")
            self._set_status((-1, "Must specify both position and stage_id"))
            current_state = "Unknown"
        else:
            # Verify move state
            current_state = self.get_state(stage_id=stage_id)
            if 'READY' not in current_state:
                self.logger.error("Move not ready for stage %d", stage_id)
                self._set_status((-1, f"Move not ready for stage {stage_id}"))
            else:
                # Verify position
                if 'absolute' not in move_type:
                    position += self.current_position[stage_id]
                if position < self.current_limits[stage_id][0] or \
                   position > self.current_limits[stage_id][1]:
                    self.logger.error("Position out of range: %s", position)
                    self._set_status((-1, f"Position out of range: {position}"))
                else:
                    ret = True
        self.logger.debug("Move state: %s", current_state)
        return ret

    def _return_parse_state(self, message=""):
        """
        Parse the return message from the controller.  The message code is
        given in the last two string characters

        :param message: String message code from the controller
        :return: String message
        """
        message = message.rstrip()
        code = message[-2:]
        return self.msg.get(code, "Unknown state")

    def __return_parse_error(self, error=""):
        """
        Parse the return error message from the controller.  The message code is
        given in the last string character

        :param error: Error code from the controller
        :return: String message
        """
        error = error.rstrip()
        code = error[-1:]
        return self.error.get(code, "Unknown error")

    def home(self, stage_id=1):
        """
        Home the stage

        :param stage_id: Int, stage position in the daisy chain starting with 1
        :return: return from _send_command
        """

        # Return value
        ret = True
        if not self.homed(stage_id):
            if self._send_command(command='OR', stage_id=stage_id):
                state = ''
                while 'READY from HOMING' not in state:
                    time.sleep(1.)
                    state = self.get_state(stage_id)
                    self.logger.debug(state)
                    if 'ERROR' in state:
                        ret = False
                        break
            else:
                ret = False
        else:
            self.logger.info("Already homed")
            self._set_status((0, "Already homed"))

        return ret

    def homed(self, stage_id=1):
        """ Is the stage homed?
        :param stage_id: Int, stage position in the daisy chain starting with 1
        :return: Boolean, True if homed else False
        """

        state = self.get_state(stage_id=stage_id)

        if 'ERROR' in state:
            self.logger.error("Error checking homed stage %d", stage_id)
            self._set_status((-1, f"Error checking homed stage {stage_id}"))
            ret = False

        else:
            if 'NOT REFERENCED' in state:
                ret = False
            else:
                ret = True

            self.logger.debug(state)

        return ret

    def move_abs(self, position=None, stage_id=None, blocking=False):
        """
        Move stage to absolute position and return when in position

        :param position: Float, absolute position in degrees
        :param stage_id: Int, stage position in the daisy chain starting with 1
        :param blocking: Boolean, block until move complete or not
        :return: return from _send_command
        """

        # Verify we are ready to move
        if self._verify_move_state(stage_id=stage_id, position=position):
            self.logger.error("Not ready to move stage %d to position: %d",
                              stage_id, position)
            self._set_status((-1, f"Not ready to move stage {stage_id} to position {position}"))
            return False

        # Send move to controller
        if self._send_command(command="PA", parameters=[position], stage_id=stage_id):

            if blocking:
                move_len = self.current_position[stage_id] - position
                if self.move_rate <= 0:
                    timeout = 5
                else:
                    timeout = int(abs(move_len / self.move_rate))
                timeout = max(timeout, 5)
                if self.logger:
                    self.logger.info("Timeout for move to absolute position: %d s",
                                     timeout)
                if self._read_blocking(stage_id=stage_id, timeout=timeout):
                    self.current_position[stage_id] = position
            return True

        self.logger.error("Error sending move command to stage %d to position: %d",
                          stage_id, position)
        self._set_status((-1, f"Error sending move command to stage {stage_id} "
                              f"to position {position}"))
        return False

    def move_rel(self, position=None, stage_id=None, blocking=False):
        """
        Move stage to relative position and return when in position

        :param position: Float, relative position in degrees
        :param stage_id: Int, stage position in the daisy chain starting with 1
        :param blocking: Boolean, block until move complete or not
        :return: return from _send_command
        """

        start = time.time()

        # Verify we are ready to move
        ret = self._verify_move_state(stage_id=stage_id, position=position,
                                      move_type='relative')
        if 'error' in ret:
            if self.logger:
                self.logger.error(ret['error'])
            return ret
        if 'OK to move' not in ret['data']:
            if self.logger:
                self.logger.error(ret['data'])
            return {'elaptime': time.time()-start, 'error': ret['data']}

        ret = self._send_command(command="PR", args=[position],
                                 stage_id=stage_id)

        if blocking:
            if self.move_rate <= 0:
                timeout = 5
            else:
                timeout = int(abs(position / self.move_rate))
            timeout = max(timeout, 5)
            if self.logger:
                self.logger.info("Timeout for move to relative position: %d s",
                                 timeout)
            ret = self._read_blocking(stage_id=stage_id, timeout=timeout)

        if 'error' not in ret:
            self.current_position[stage_id] += position

        ret['elaptime'] = time.time() - start
        return ret

    def get_state(self, stage_id=1) -> str:
        """ Current state of the stage

        :param stage_id: int, stage position in the daisy chain starting with 1
        :return: return from _send_command
        """

        if self._send_command(command="TS", stage_id=stage_id):
            ret = self._return_parse_state(self._read_reply())
        else:
            ret = "ERROR"

        return ret

    def get_last_error(self, stage_id=1):
        """ Last error

        :param stage_id: int, stage position in the daisy chain starting with 1
        :return: return from _send_command
        """

        start = time.time()

        ret = self._send_command(command="TE", stage_id=stage_id)
        if 'error' not in ret:
            last_error = self.__return_parse_error(self._read_reply())
            ret['data'] = last_error
            ret['elaptime'] = time.time() - start

        return ret

    def get_position(self, stage_id=1):
        """ Current position

        :param stage_id: int, stage position in the daisy chain starting with 1
        :return: return from _send_command
        """

        start = time.time()

        ret = self._send_command(command="TP", stage_id=stage_id)
        if 'error' not in ret:
            position = float(self._read_reply().rstrip()[3:])
            self.current_position[stage_id] = position
            ret['data'] = position
            ret['elaptime'] = time.time() - start

        return ret

    def get_move_rate(self):
        """ Current move rate

        :return: return from _send_command
        """
        start = time.time()
        return {'elaptime': time.time()-start, 'data': self.move_rate}

    def set_move_rate(self, rate=5.0):
        """ Set move rate

        :param rate: Float, move rate in degrees per second
        :return: dictionary {'elaptime': time, 'data': move_rate}
        """
        start = time.time()
        if rate > 0:
            self.move_rate = rate
        else:
            if self.logger:
                self.logger.error('set_move_rate input error, not changed')
        return {'elaptime': time.time()-start, 'data': self.move_rate}

    def reset(self, stage_id=1):
        """ Reset stage

        :param stage_id: int, stage position in the daisy chain starting with 1
        :return: return from _send_command
        """

        start = time.time()

        ret = self._send_command(command="RS", stage_id=stage_id)
        time.sleep(2.)

        if 'error' not in ret:
            self.read_from_controller()

        ret['elaptime'] = time.time() - start
        return ret

    def get_limits(self, stage_id=1):
        """ Get stage limits
        :param stage_id: int, stage position in the daisy chain starting with 1
        :return: return from _send_command
        """
        start = time.time()
        ret = self._send_command(command="SL", parameters="?", stage_id=stage_id)
        if 'error' not in ret:
            lolim = int(self._read_reply().rstrip()[3:])
            ret = self._send_command(command="SR", parameters="?", stage_id=stage_id)
            if 'error' not in ret:
                uplim = int(self._read_reply().rstrip()[3:])
                self.current_limits[stage_id] = (lolim, uplim)
                ret = {'elaptime': time.time()-start,
                       'data': self.current_limits[stage_id]}
        return ret

    def get_params(self, stage_id=1, quiet=False):
        """ Get stage parameters

        :param stage_id: int, stage position in the daisy chain starting with 1
        :param quiet: Boolean, do not print parameters
        :return: return from _send_command
        """

        start = time.time()

        ret = self._send_command(command="ZT", stage_id=stage_id)

        if 'error' not in ret:
            params = self._read_params()
            if not quiet:
                for param in params.split():
                    if 'PW' not in param:
                        print(param)
            ret['data'] = params
            ret['elaptime'] = time.time() - start

        return ret

    def initialize_controller(self):
        """ Initialize stage controller. """
        start = time.time()
        for i in range(self.num_stages):
            self.get_position(i+1)
            self.get_limits(i+1)
        return {'elaptime': time.time()-start, 'data': 'initialized'}

    def read_from_controller(self):
        """ Read from controller"""
        self.socket.setblocking(False)
        try:
            recv = self.socket.recv(2048)
            recv_len = len(recv)
            if self.logger:
                self.logger.debug("Return: len = %d, Value = %s", recv_len, recv)
        except BlockingIOError:
            recv = b""
        self.socket.setblocking(True)
        return str(recv.decode('utf-8'))

    def run_manually(self, stage_id=1):
        """ Input stage commands manually

        :param stage_id: int, stage position in the daisy chain starting with 1
        :return: None
        """

        while True:

            cmd = input("Enter Command")

            if not cmd:
                break

            ret = self._send_command(command=cmd, stage_id=stage_id,
                                     custom_command=True)
            if 'error' not in ret:
                output = self.read_from_controller()
                print(output)

            if self.logger:
                self.logger.debug("End: %s", ret)
