import serial
import struct
import time
import threading
import warnings

from .message import Message
PORT_GP2 = 0x01
MODE_PTP_JUMP_XYZ = 0x00
MODE_PTP_MOVJ_XYZ = 0x01
MODE_PTP_MOVL_XYZ = 0x02
MODE_PTP_JUMP_ANGLE = 0x03
MODE_PTP_MOVJ_ANGLE = 0x04
MODE_PTP_MOVL_ANGLE = 0x05
MODE_PTP_MOVJ_INC = 0x06
MODE_PTP_MOVL_INC = 0x07
MODE_PTP_MOVJ_XYZ_INC = 0x08
MODE_PTP_JUMP_MOVL_XYZ = 0x09

GET_SET_DEVICE_SN = 0
GET_SET_DEVICE_NAME = 1
GET_POSE = 10
RESET_POSE = 11
GET_ALARMS_STATE = 20
CLEAR_ALL_ALARMS_STATE = 21
SET_GET_HOME_PARAMS = 30
SET_HOME_CMD = 31
SET_GET_HHTTRIG_MODE = 40
SET_GET_HHTTRIG_OUTPUT_ENABLED = 41
GET_HHTTRIG_OUTPUT = 42
SET_GET_ARM_ORIENTATION = 50
SET_GET_END_EFFECTOR_PARAMS = 60
SET_GET_END_EFFECTOR_LAZER = 61
SET_GET_END_EFFECTOR_SUCTION_CUP = 62
SET_GET_END_EFFECTOR_GRIPPER = 63
SET_GET_JOG_JOINT_PARAMS = 70
SET_GET_JOG_COORDINATE_PARAMS = 71
SET_GET_JOG_COMMON_PARAMS = 72
SET_GET_PTP_JOINT_PARAMS = 80
SET_GET_PTP_COORDINATE_PARAMS = 81
SET_GET_PTP_JUMP_PARAMS = 82
SET_GET_PTP_COMMON_PARAMS = 83
SET_PTP_CMD = 84
SET_CP_CMD = 91
SET_QUEUED_CMD_START_EXEC = 240
SET_QUEUED_CMD_STOP_EXEC = 241
SET_QUEUED_CMD_CLEAR = 245
GET_QUEUED_CMD_CURRENT_INDEX = 246

IO_MODES = {'Dummy': 1, 'PWM': 2, 'DO': 3, 'DI': 4, 'ADC': 5}


class Dobot:

    def __init__(self, port, verbose=False):
        threading.Thread.__init__(self)

        self._on = True
        self.verbose = verbose
        self.lock = threading.Lock()
        self.ser = serial.Serial(port,
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)
        is_open = self.ser.isOpen()
        if self.verbose:
            print('pydobot: %s open' % self.ser.name if is_open else 'failed to open serial port')

        self._set_queued_cmd_start_exec()
        self._set_queued_cmd_clear()
        self._set_ptp_joint_params(200, 200, 200, 200, 200, 200, 200, 200)
        self._set_ptp_coordinate_params(velocity=200, acceleration=200)
        self._set_ptp_jump_params(200, 200)
        self._set_ptp_common_params(velocity=50, acceleration=50)
        self._get_pose()

    """
        Gets the current command index
    """

    def _get_queued_cmd_current_index(self):
        msg = Message()
        msg.id = GET_QUEUED_CMD_CURRENT_INDEX
        response = self._send_command(msg)
        idx = struct.unpack_from('L', response.params, 0)[0]
        return idx

    """
        Gets the real-time pose of the Dobot
    """

    def _get_pose(self):
        msg = Message()
        msg.id = 10
        response = self._send_command(msg)
        self.x = struct.unpack_from('f', response.params, 0)[0]
        self.y = struct.unpack_from('f', response.params, 4)[0]
        self.z = struct.unpack_from('f', response.params, 8)[0]
        self.r = struct.unpack_from('f', response.params, 12)[0]
        self.j1 = struct.unpack_from('f', response.params, 16)[0]
        self.j2 = struct.unpack_from('f', response.params, 20)[0]
        self.j3 = struct.unpack_from('f', response.params, 24)[0]
        self.j4 = struct.unpack_from('f', response.params, 28)[0]

        if self.verbose:
            print("pydobot: x:%03.1f \
                            y:%03.1f \
                            z:%03.1f \
                            r:%03.1f \
                            j1:%03.1f \
                            j2:%03.1f \
                            j3:%03.1f \
                            j4:%03.1f" %
                  (self.x, self.y, self.z, self.r, self.j1, self.j2, self.j3, self.j4))
        return response

    def _read_message(self):
        time.sleep(0.1)
        b = self.ser.read_all()
        if len(b) > 0:
            msg = Message(b)
            if self.verbose:
                print('pydobot: <<', msg)
            return msg
        return

    def _send_command(self, msg, wait=False):
        self.lock.acquire()
        self._send_message(msg)
        response = self._read_message()
        self.lock.release()

        if not wait:
            return response

        expected_idx = struct.unpack_from('L', response.params, 0)[0]
        if self.verbose:
            print('pydobot: waiting for command', expected_idx)

        while True:
            current_idx = self._get_queued_cmd_current_index()

            if current_idx != expected_idx:
                time.sleep(0.1)
                continue

            if self.verbose:
                print('pydobot: command %d executed' % current_idx)
            break

        return response

    def _send_message(self, msg):
        time.sleep(0.1)
        if self.verbose:
            print('pydobot: >>', msg)
        self.ser.write(msg.bytes())

    """
        Executes the CP Command
    """

    def _set_cp_cmd(self, x, y, z):
        msg = Message()
        msg.id = SET_CP_CMD
        msg.ctrl = 3
        msg.params = bytearray(bytes([0x01]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.append(0x00)
        return self._send_command(msg)

    """
        Sets the status of the gripper
    """

    def _set_end_effector_gripper(self, enable=False):
        msg = Message()
        msg.id = SET_GET_END_EFFECTOR_GRIPPER
        msg.ctrl = 3
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    """
        Sets the status of the suction cup
    """

    def _set_end_effector_suction_cup(self, enable=False):
        msg = Message()
        msg.id = SET_GET_END_EFFECTOR_SUCTION_CUP
        msg.ctrl = 3
        msg.params = bytearray([])
        msg.params.extend(bytearray([0x01]))
        if enable is True:
            msg.params.extend(bytearray([0x01]))
        else:
            msg.params.extend(bytearray([0x00]))
        return self._send_command(msg)

    """
        Sets the velocity ratio and the acceleration ratio in PTP mode
    """

    def _set_ptp_joint_params(self, v_x, v_y, v_z, v_r, a_x, a_y, a_z, a_r):
        msg = Message()
        msg.id = SET_GET_PTP_JOINT_PARAMS
        msg.ctrl = 3
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', v_x)))
        msg.params.extend(bytearray(struct.pack('f', v_y)))
        msg.params.extend(bytearray(struct.pack('f', v_z)))
        msg.params.extend(bytearray(struct.pack('f', v_r)))
        msg.params.extend(bytearray(struct.pack('f', a_x)))
        msg.params.extend(bytearray(struct.pack('f', a_y)))
        msg.params.extend(bytearray(struct.pack('f', a_z)))
        msg.params.extend(bytearray(struct.pack('f', a_r)))
        return self._send_command(msg)

    """
        Sets the velocity and acceleration of the Cartesian coordinate axes in PTP mode
    """

    def _set_ptp_coordinate_params(self, velocity, acceleration):
        msg = Message()
        msg.id = SET_GET_PTP_COORDINATE_PARAMS
        msg.ctrl = 3
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    """
       Sets the lifting height and the maximum lifting height in JUMP mode
    """

    def _set_ptp_jump_params(self, jump, limit):
        msg = Message()
        msg.id = SET_GET_PTP_JUMP_PARAMS
        msg.ctrl = 3
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', jump)))
        msg.params.extend(bytearray(struct.pack('f', limit)))
        return self._send_command(msg)

    """
        Sets the velocity ratio, acceleration ratio in PTP mode
    """

    def _set_ptp_common_params(self, velocity, acceleration):
        msg = Message()
        msg.id = SET_GET_PTP_COMMON_PARAMS
        msg.ctrl = 3
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('f', velocity)))
        msg.params.extend(bytearray(struct.pack('f', acceleration)))
        return self._send_command(msg)

    """
        Executes PTP command
    """

    def _set_ptp_cmd(self, x, y, z, r, mode, wait):
        msg = Message()
        msg.id = 84
        msg.ctrl = 3
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        return self._send_command(msg, wait)

    """
        Clears command queue
    """

    def _set_queued_cmd_clear(self):
        msg = Message()
        msg.id = SET_QUEUED_CMD_CLEAR
        msg.ctrl = 1
        return self._send_command(msg)

    """
        Start command
    """

    def _set_queued_cmd_start_exec(self):
        msg = Message()
        msg.id = SET_QUEUED_CMD_START_EXEC
        msg.ctrl = 1
        return self._send_command(msg)

    """
        Stop command
    """

    def _set_queued_cmd_stop_exec(self):
        msg = Message()
        msg.id = SET_QUEUED_CMD_STOP_EXEC
        msg.ctrl = 1
        return self._send_command(msg)

    def close(self):
        self._on = False
        self.lock.acquire()
        self.ser.close()
        if self.verbose:
            print('pydobot: %s closed' % self.ser.name)
        self.lock.release()

    def go(self, x, y, z, r=0.):
        warnings.warn('go() is deprecated, use move_to() instead')
        self.move_to(x, y, z, r)

    def move_to(self, x, y, z, r, wait=False):
        self._set_ptp_cmd(x, y, z, r, mode=MODE_PTP_MOVJ_XYZ, wait=wait)

    def suck(self, enable):
        self._set_end_effector_suction_cup(enable)

    def grip(self, enable):
        self._set_end_effector_gripper(enable)

    def speed(self, velocity=100., acceleration=100.):
        self._set_ptp_common_params(velocity, acceleration)
        self._set_ptp_coordinate_params(velocity, acceleration)

    def pose(self):
        response = self._get_pose()
        x = struct.unpack_from('f', response.params, 0)[0]
        y = struct.unpack_from('f', response.params, 4)[0]
        z = struct.unpack_from('f', response.params, 8)[0]
        r = struct.unpack_from('f', response.params, 12)[0]
        j1 = struct.unpack_from('f', response.params, 16)[0]
        j2 = struct.unpack_from('f', response.params, 20)[0]
        j3 = struct.unpack_from('f', response.params, 24)[0]
        j4 = struct.unpack_from('f', response.params, 28)[0]
        return x, y, z, r, j1, j2, j3, j4

    def wait(self, ms, wait=False):
        self._set_wait_cmd(ms, wait)

    def start_stepper(self, pps, motor=0, wait=False):
        self._set_emotor(motor, 1, pps, wait)

    def stop_stepper(self, motor=0, wait=False):
        self._set_emotor(motor, 0, 0, wait)

    def start_conveyor(self, speed, motor=0, wait=False):
        self._set_emotor(motor, 1, int(19800 * speed), wait)  #

    def set_io_mode(self, address, mode, wait=False):
        self._set_io_multiplexing(address, IO_MODES[mode], wait)

    def set_pwm_output(self, address, frequency, duty_cycle, wait=False):
        self._set_io_pwm(address, frequency, duty_cycle, wait)

    #####
    def _set_ptp_withL_cmd(self, x, y, z, r, l, mode, wait):
        msg = Message()
        msg.id = 86
        msg.ctrl = 3
        msg.params = bytearray([])
        msg.params.extend(bytearray([mode]))
        msg.params.extend(bytearray(struct.pack('f', x)))
        msg.params.extend(bytearray(struct.pack('f', y)))
        msg.params.extend(bytearray(struct.pack('f', z)))
        msg.params.extend(bytearray(struct.pack('f', r)))
        msg.params.extend(bytearray(struct.pack('f', l)))

        return self._send_command(msg, wait)

    def move_to_withL(self, x, y, z, r, l, wait=True):
        self._set_ptp_withL_cmd(x, y, z, r, l, mode=0x02, wait=wait)

    def move_conveyor(self, distance, direction, motor=0, speed=6000, wait=False):
        # distance in cm
        # direction: 0=forward, 1=backward

        if direction == 1:
            speed = speed * -1
        self.start_stepper(speed, motor, wait)  # cca 5cm/sec
        self.wait(228 * distance)
        self.stop_stepper(motor, wait)

    def enable_rail(self, wait=False):
        msg = Message()
        msg.id = 3
        msg.ctrl = 3
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('i', int(True))))
        return self._send_command(msg, wait)

    def _set_color_sensor(self, state, port=0x01, wait=False):
        msg = Message()
        msg.id = 137
        msg.ctrl = 0x03
        msg.params = bytearray([])
        msg.params.extend(bytearray([int(state)]))
        msg.params.extend(bytearray([port]))
        msg.params.extend(bytearray([1]))
        return self._send_command(msg, True)

    def enable_color_sensor(self, port=0x01, wait=False):
        self._set_color_sensor(1, port, wait)

    def disable_color_sensor(self, port=0x01, wait=False):
        self._set_color_sensor(0, port, wait)

    def read_color(self, port=0x01, wait=False):
        msg = Message()
        msg.id = 137
        msg.ctrl = 0
        msg.params = bytearray([])
        msg.params.extend(bytearray(struct.pack('i', port)))
        response = self._send_command(msg, wait)
        r = struct.unpack_from('?', response.params, 0)[0]
        g = struct.unpack_from('?', response.params, 1)[0]
        b = struct.unpack_from('?', response.params, 2)[0]

        return [r, g, b]

    def _set_emotor(self, index, enabled, speed, wait=False):
        msg = Message()
        msg.id = 135
        msg.ctrl = 0x03
        msg.params = bytearray(struct.pack('B', index))
        msg.params.extend(bytearray(struct.pack('B', enabled)))
        msg.params.extend(bytearray(struct.pack('i', speed)))
        return self._send_command(msg, wait=wait)

    def _set_wait_cmd(self, ms, wait):
        msg = Message()
        msg.id = 110
        msg.ctrl = 0x03
        msg.params = bytearray(struct.pack('I', ms))
        return self._send_command(msg, wait=wait)

    def home(self, wait=False):
        msg = Message()
        msg.id = 31
        msg.ctrl = 0x03
        self._send_command(msg, True)


class CommunicationProtocolIDs():
    GET_SET_DEVICE_SN = 0
    GET_SET_DEVICE_NAME = 1
    GET_POSE = 10
    RESET_POSE = 11
    GET_ALARMS_STATE = 20
    CLEAR_ALL_ALARMS_STATE = 21
    SET_GET_HOME_PARAMS = 30
    SET_HOME_CMD = 31
    SET_GET_HHTTRIG_MODE = 40
    SET_GET_HHTTRIG_OUTPUT_ENABLED = 41
    GET_HHTTRIG_OUTPUT = 42
    SET_GET_ARM_ORIENTATION = 50
    SET_GET_END_EFFECTOR_PARAMS = 60
    SET_GET_END_EFFECTOR_LAZER = 61
    SET_GET_END_EFFECTOR_SUCTION_CUP = 62
    SET_GET_END_EFFECTOR_GRIPPER = 63
    SET_GET_JOG_JOINT_PARAMS = 70
    SET_GET_JOG_COORDINATE_PARAMS = 71
    SET_GET_JOG_COMMON_PARAMS = 72
    SET_GET_PTP_JOINT_PARAMS = 80
    SET_GET_PTP_COORDINATEP_ARAMS = 81
    SET_GET_PTP_JUMP_PARAMS = 82
    SET_GET_PTP_COMMON_PARAMS = 83
    SET_PTP_CMD = 84
    SET_CP_CMD = 91
    SET_QUEUED_CMD_START_EXEC = 240
    SET_QUEUED_CMD_STOP_EXEC = 241
    SET_QUEUED_CMD_CLEAR = 245
    GET_QUEUED_CMD_CURRENT_INDEX = 246
