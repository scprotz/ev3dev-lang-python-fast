#!/usr/bin/env micropython
import sys
import os
import select
import time

# set up some logging so we can see errors #
import logging
logging.basicConfig()
logger = logging.getLogger("ev3fast_micropython")


#####################################################################################

class Device:
    """ Generic Device class that allows for reading and writing to a given
        device and its attribute files.
    """
    _directory = ""

    def __init__(self):
        pass

    def readStr(self, attribute):
        logger.debug("Reading from " + self._directory + attribute)
        try:
            value = ""
            f = open(self._directory + attribute, 'r')
            value = f.read().strip()
            logger.debug("Read value " + str(value) +  str(type(value)))
            f.close()
            return value
        except OSError as e:
            raise Exception("Could not read from \"" + self._directory + attribute + "\" .")

    def readInt(self, attribute):
        value = float('NaN')
        try:
            value = int(self.readStr(attribute))
        except:
            raise Exception("Could not read int value: " + self._directory + attribute)
        return value

    def readbin(self, attribute):
        logger.debug("Reading from " + self._directory + attribute)
        try:
            value = ""
            f = open(self._directory + attribute, 'rb')
            value = f.read()
            logger.debug("Read value " + str(value) +  str(type(value)))
            f.close()
            return value
        except OSError as e:
            raise Exception("Could not read from \"" + self._directory + attribute + "\" .")

    def write(self, attribute, value):
        logger.debug("Writing to " + self._directory + attribute + " with value " + str(value))
        f = open(self._directory + attribute, 'w')
        f.write(str(value))
        f.close()

class Sensor(Device):
    _DIRECTORY_BASE = '/sys/class/lego-sensor/'
    _DRIVER_NAME = None

    

    def __init__(self, address=None):
        # most sensors have a mode, so keep track of it
        self._currentMode = None

        filenames = os.listdir(self._DIRECTORY_BASE)

        for file in filenames:
            directory = self._DIRECTORY_BASE + file + '/'

            # set path to this directory to test
            self._directory = directory

            if address:
                # get the address for this sensor
                sensor_address = self.readStr('address')

                # if it is the correct one, break
                if address in sensor_address:
                    break
                # not this one, so set path back to empty
                self._directory = None

            else:
                # get the driver name from this sensor
                sensor_driver_name = self.readStr('driver_name').strip()

                # if it is the correct one, break
                if sensor_driver_name in self._DRIVER_NAME:
                    break

                # not this one, so set path back to empty
                self._directory = None

        if self._directory == None:
            if address:
                raise Exception("Could not find sensor at '" + address + "'")
            else:
                raise Exception("Could not find a sensor of type " + self._DRIVER_NAME)

        self.address = self.readStr('address')
        self.driver_name = self.readStr('driver_name')
        self.modes = self.readStr('modes')

        self._currentMode = self.readStr('mode')

#   def bin_data(self, fmt=None):
#     value_size = self._BYTES_FMT[self.bin_data_format][0]
#     num_bytes = self.num_values * value_size
#     data = os.read(self._fd['bin_data'], num_bytes)
#     if fmt:
#       return struct.unpack(fmt, data)
#     else:
#       return bytearray(data)

#   @property
#   def bin_data_format(self):
#     if self._bin_data_format_mode != self._currentMode:
#       os.lseek(self._fd['bin_data_format'], 0, os.SEEK_SET)
#       self._bin_data_format = os.read(
#           self._fd['bin_data_format'], 100).decode('utf-8').rstrip()
#       self._bin_data_format_mode = self._currentMode
#     return self._bin_data_format

    @property
    def command(self):
        raise Exception("command is a write-only property!")

    @command.setter
    def command(self, value):
        self.write('command', value)

    @property
    def commands(self):
        return self.read('commands')
    
    @property
    def decimals(self):
        if self._decimals_mode != self._currentMode:
            self._decimals = self.readInt('decimals')
            self._decimals_mode = self._currentMode
        return self._decimals

    @property
    def mode(self):
        return self._currentMode.strip()

    @mode.setter
    def mode(self, mode):
        if (self._currentMode != mode):
            self.write('mode', mode)
            self._currentMode = mode
        return

    @property
    def num_values(self):
        if self._num_values_mode != self._currentMode:
            self._num_values = self.readInt('num_values')            
            self._num_values_mode = self._currentMode
        return self._num_values

    @property
    def units(self):
        if self._units_mode != self._currentMode:
            self._units = self.readStr('units')
            self._units_mode = self._currentMode
        return self._units

#   def value(self, n=0):
#     fmt = self._BYTES_FMT[self.bin_data_format]
#     os.lseek(self._fd['bin_data'], 0, os.SEEK_SET)
#     startByte = n * fmt[0]
#     endByte = startByte + fmt[0]
#     data = os.read(self._fd['bin_data'], endByte)
#     return struct.unpack(fmt[1], data[startByte:endByte])[0]


class TouchSensor(Sensor):
    _DRIVER_NAME = 'lego-ev3-touch'
    MODE_TOUCH = 'TOUCH'
  
    def __init__(self, address=None):
        super().__init__(address)

    @property
    def is_pressed(self):
        value = self.readInt("value0")
        if value == 1:
            return True
        return False

    @property
    def is_released(self):
        return not self.is_pressed

    def _wait(self, desired_state, timeout_ms, sleep_ms):
        tic = time.time()

        if sleep_ms:
            sleep_ms = float(sleep_ms / 1000)

        while True:
            if self.is_pressed == desired_state:
                return True
      
            if timeout_ms is not None and time.time() >= tic + timeout_ms / 1000:
                return False
      
            if sleep_ms:
                time.sleep(sleep_ms)
  
    def wait_for_pressed(self, timeout_ms=None, sleep_ms=10):
        return self._wait(True, timeout_ms, sleep_ms)
    
    def wait_for_released(self, timeout_ms=None, sleep_ms=10):
        return self._wait(False, timeout_ms, sleep_ms)
    
    def wait_for_bump(self, timeout_ms=None, sleep_ms=10):
        start_time = time.time()

        if self.wait_for_pressed(timeout_ms, sleep_ms):
            if timeout_ms is not None:
                timeout_ms -= int((time.time() - start_time) * 1000)
            return self.wait_for_pressed(timeout_ms, sleep_ms)
        return False

class ColorSensor(Sensor):
    _DRIVER_NAME = 'lego-ev3-color'

    COLOR_NOCOLOR = 0
    COLOR_BLACK = 1
    COLOR_BLUE = 2
    COLOR_GREEN = 3
    COLOR_YELLOW = 4
    COLOR_RED = 5
    COLOR_WHITE = 6
    COLOR_BROWN = 7

    COLORS = (
      'NoColor',
      'Black',
      'Blue',
      'Green',
      'Yellow',
      'Red',
      'White',
      'Brown',
    )

    MODE_COL_REFLECT = 'COL-REFLECT'
    MODE_COL_AMBIENT = 'COL-AMBIENT'
    MODE_COL_COLOR = 'COL-COLOR'
    MODE_REF_RAW = 'REF-RAW'
    MODE_RGB_RAW = 'RGB-RAW'

    def __init__(self, address=None):
        self.red_max = 300
        self.green_max = 300
        self.blue_max = 300
        super().__init__(address)

    @property
    def reflected_light_intensity(self):
        if (self._currentMode != self.MODE_COL_REFLECT):
            self.mode = self.MODE_COL_REFLECT
        value0 = self.readInt('value0')
        return value0

    @property
    def ambient_light_intensity(self):
        if (self._currentMode != self.MODE_COL_AMBIENT):
            self.mode = self.MODE_COL_AMBIENT
        value0 = self.readInt('value0')
        return value0

    @property
    def color(self):
        if (self._currentMode != self.MODE_COL_COLOR):
            self.mode = self.MODE_COL_COLOR
        value0 = self.readInt('value0')
        return value0

    @property
    def color_name(self):
        return self.COLORS[self.color]

    @property
    def raw(self):
        if (self._currentMode != self.MODE_RGB_RAW):
            self.mode = self.MODE_RGB_RAW
        value0 = self.readInt('value0')
        value1 = self.readInt('value1')
        value2 = self.readInt('value2')
        return (value0, value1, value2)
  
    @property
    def calibrate_white(self):
        (self.red_max, self.green_max, self.blue_max) = self.raw

    @property
    def rgb(self):
        (red, green, blue) = self. raw
        
        return (
            int(red * 255 / self.red_max),
            int(green * 255 / self.green_max),
            int(blue * 255 / self.blue_max)
        )

    @property
    def red(self):
        if (self._currentMode != self.MODE_RGB_RAW):
            self.mode = self.MODE_RGB_RAW
        return self.raw[0]

    @property
    def green(self):
        if (self._currentMode != self.MODE_RGB_RAW):
            self.mode = self.MODE_RGB_RAW
        return self.raw[1]

    @property
    def blue(self):
        if (self._currentMode != self.MODE_RGB_RAW):
            self.mode = self.MODE_RGB_RAW
        return self.raw[2]

class UltrasonicSensor(Sensor):
    _DRIVER_NAME = ['lego-ev3-us', 'lego-nxt-us']

    MODE_US_DIST_CM = 'US-DIST-CM'
    MODE_US_DIST_IN = 'US-DIST-IN'
    MODE_US_LISTEN = 'US-LISTEN'
    MODE_US_SI_CM = 'US-SI-CM'
    MODE_US_SI_IN = 'US-SI-IN'
    
    @property
    def distance_centimeters(self):
        if (self._currentMode != self.MODE_US_DIST_CM):
            self.mode = self.MODE_US_DIST_CM
        value = self.readInt('value0')
        if self.driver_name == "lego-nxt-us":
            return value
        return value/10

    @property
    def distance_inches(self):
        if (self._currentMode != self.MODE_US_DIST_IN):
            self.mode = self.MODE_US_DIST_IN
        return self.readInt('value0')/10
        
    @property
    def other_sensor_present(self):
        if (self._currentMode != self.MODE_US_LISTEN):
            self.mode = self.MODE_US_LISTEN
        return self.readInt('value0')

class GyroSensor(Sensor):
    _DRIVER_NAME = 'lego-ev3-gyro'

    MODE_GYRO_ANG = 'GYRO-ANG'
    MODE_GYRO_CAL = 'GYRO-CAL'
    MODE_GYRO_FAS = 'GYRO-FAS'
    MODE_GYRO_G_A = 'GYRO-G&A'
    MODE_GYRO_RATE = 'GYRO-RATE'
    MODE_TILT_ANG = 'TILT-ANGLE'
    MODE_TILT_RATE = 'TILT-RATE'  
    
    @property
    def angle(self):
        if (self._currentMode != self.MODE_GYRO_ANG):
            self.mode = self.MODE_GYRO_ANG
        return self.readInt('value0')

    @property
    def rate(self):
        if (self._currentMode != self.MODE_GYRO_RATE):
            self.mode = self.MODE_GYRO_RATE
        return self.readInt('value0')

    @property
    def angle_and_rate(self):
        if (self._currentMode != self.MODE_GYRO_G_A):
            self.mode = self.MODE_GYRO_G_A
        value0 = self.readInt('value0')
        value1 = self.readInt('value1')
        return (value0, value1)

class SpeedValue:
    """
    The base class for the SpeedValue classes.
    Not meant to be used directly.
    Use SpeedNativeUnits, SpeedPercent, SpeedRPS, SpeedRPM, SpeedDPS, SpeedDPM
    """

    def __lt__(self, other):
        return self.to_native_units() < other.to_native_units()
    
    def __rmul__(self, other):
        return self.__mul__(other)
    
    def to_native_units(self):
        pass
    
    def __mul__(self, other):
        pass

class SpeedPercent(SpeedValue):
    """
    Speed as a percentage of the motor's maximum rated speed.
    Returns Tacho Counts via motor.max_speed
    """

    def __init__(self, percent):
        if -100 <= percent <= 100:
            self.percent = percent
        else:
            raise ValueError("Value must be between -100 and 100")

    def __str__(self):
        return str(self.percent) + '%'

    def __mul__(self, other):
        if isinstance(other, (float, int)):
            return SpeedPercent(self.percent * other)
        else:
            raise TypeError("Multiplier must be of int or float type.")

    def to_native_units(self, motor):
        return self.percent / 100 * motor.max_speed

class SpeedNativeUnits(SpeedValue):
    """
    Speed in tacho counts per second
    Returns no. of Tacho Counts
    """

    def __init__(self, native_counts):
        self.native_counts = native_counts

    def __str__(self):
        return str(self.native_counts) + "counts/sec"

    def __mul__(self, other):
        if isinstance(other, (float, int)):
            SpeedNativeUnits(self.native_counts * other)
        else:
            raise TypeError("Multiplier must be of int or float type.")

    def to_native_units(self, motor):
        return self.native_counts

class SpeedRPS(SpeedValue):
    """
    Speed in rotations-per-second
    Returns Tacho Counts via motor.max_rps
    """

    def __init__(self, rotations_per_second):
        self.rotations_per_second = rotations_per_second

    def __str__(self):
        return str(self.rotations_per_second) + " rot/sec"
    
    def __mul__(self, other):
        if isinstance(other, (float, int)):
            return SpeedRPS(self.rotations_per_second * other)
        else:
            raise TypeError("Multiplier must be of float or int type")

    def to_native_units(self, motor):
        if abs(self.rotations_per_second) <= motor.max_rps:
            return self.rotations_per_second / motor.max_rps * motor.max_speed
        else:
            raise ValueError("RPS value must be <= motor.max_rps")

class SpeedRPM(SpeedValue):
    """
    Speed in rotations-per-minute
    Returns RPM to Tacho Counts via motor.max_rpm
    """

    def __init__(self, rotations_per_minute):
        self.rotations_per_minute = rotations_per_minute

    def __str__(self):
        return str(self.rotations_per_minute) + " rot/min"
        
    def __mul__(self, other):
        if isinstance(other, (float, int)):
            return SpeedRPM(self.rotations_per_minute * other)
        else:
            raise TypeError("Multiplier must be of type float or int") 

    def to_native_units(self, motor):
        if abs(self.rotations_per_minute) <= motor.max_rpm:
            return self.rotations_per_minute / motor.max_rpm * motor.max_speed
        else:
            raise ValueError("RPM Value must be <= motor.max_rpm")
    
class SpeedDPS(SpeedValue):
    """
    Speed in degrees-per-second.
    Converts to Tacho Counts via motor.max_dps
    """

    def __init__(self, degrees_per_second):
        self.degrees_per_second = degrees_per_second

    def __str__(self):
        return str(self.degrees_per_second) + " deg/sec"

    def __mul__(self, other):
        if isinstance(other, (float, int)):
            return SpeedDPS(self.degrees_per_second * other)
        else:
            raise TypeError("Multiplier must be of type float or int")

    def to_native_units(self, motor):
        if abs(self.degrees_per_second) <= motor.max_dps:
            return self.degrees_per_second / motor.max_dps * motor.max_speed
        else:
            raise ValueError("DPS Value must be <= motor.max_dps")

class SpeedDPM(SpeedValue):
    """
    Speed in degrees-per-min
    Returns Tacho Counts via motor.max_dpm
    """

    def __init__(self, degrees_per_minute):
        self.degrees_per_minute = degrees_per_minute

    def __str__(self):
        return str(self.degrees_per_minute) + " deg/min"

    def __mul__(self, other):
        if isinstance(other, (float, int)):
            return SpeedDPM(self.degrees_per_minute * other)
        else:
            raise TypeError("Multiplier must be of type float or int")

    def to_native_units(self, motor):
        if abs(self.degrees_per_minute) <= motor.max_dpm:
            return self.degrees_per_minute / motor.max_dpm * motor.max_speed
        else:
            raise ValueError("DPM Value must be <= motor.max_dpm")

class Motor(Device):
    COMMAND_RESET = 'reset'
    COMMAND_RUN_DIRECT = 'run-direct'
    COMMAND_RUN_FOREVER = 'run-forever'
    COMMAND_RUN_TIMED = 'run-timed'
    COMMAND_RUN_TO_ABS_POS = 'run-to-abs-pos'
    COMMAND_RUN_TO_REL_POS = 'run-to-rel-pos'
    COMMAND_STOP = 'stop'
    ENCODER_POLARITY_INVERSED = 'inversed'
    ENCODER_POLARITY_NORMAL = 'normal'
    POLARITY_INVERSED = 'inversed'
    POLARITY_NORMAL = 'normal'
    STATE_HOLDING = 'holding'
    STATE_OVERLOADED = 'overloaded'
    STATE_RAMPING = 'ramping'
    STATE_RUNNING = 'running'
    STATE_STALLED = 'stalled'
    STOP_ACTION_BRAKE = 'brake'
    STOP_ACTION_COAST = 'coast'
    STOP_ACTION_HOLD = 'hold'

    WAIT_RUNNING_TIMEOUT = 100

    _DIRECTORY_BASE = '/sys/class/tacho-motor/'

    _attributes = [
        ('address', str),
        ('commands', str),
        ('count_per_rot', int),
        ('count_per_m', int),
        ('full_travel_count', int),
        ('driver_name', str),
        ('max_speed', int),
        ('stop_actions', str)
    ]
    
    _DRIVER_NAME = None

    speed_sp_table = []
    for i in range(0, 1561):
        speed_sp_table.append(str(i))

    def __init__(self, address=None):
       

        filenames = os.listdir(self._DIRECTORY_BASE)
        for file in filenames:
            directory = self._DIRECTORY_BASE + file + '/'  

            logger.debug("Searching for motor at " + directory)
            # set path to this directory to test
            self._directory = directory

            if address:
                # get the address for this motor
                motor_address = self.readStr('address')

                # if it is the correct one, break
                if address in motor_address:
                    break
                # not this one, so set path back to empty
                self._directory = None

            else:
                # get the driver name from this motor
                motor_driver_name = self.readStr('driver_name').strip()

                # if it is the correct one, break
                if self._DRIVER_NAME == motor_driver_name:
                    break

                # not this one, so set path back to empty
                self._directory = None
        
        if self._directory == None:
            if address:
                raise Exception("Could not find motor at '" + address + "'")
            else:
                raise Exception("Could not find a motor of type " + self._DRIVER_NAME)
        
        self._load_attributes()
        self._state_fd = os.open(self._directory + "state", os.O_RDONLY)

        self.max_rps = float(self.max_speed / self.count_per_rot)
        self.max_rpm = self.max_rps * 60
        self.max_dps = self.max_rps * 360
        self.max_dpm = self.max_rpm * 360

    def _load_attributes(self):
        for attribute in self._attributes:
            try:
                value = self.readStr(attribute[0])
                setattr(self, attribute[0], attribute[1](value))
            except:
                pass

    @property
    def command(self):
        raise Exception("command is a write-only property!")

    @command.setter
    def command(self, value):
        self.write('command', value)
        return 0
  
    @property
    def duty_cycle(self):
        return self.readInt('duty_cycle')

    @property
    def duty_cycle_sp(self):
        self.readInt('duty_cycle_sp')

    @duty_cycle_sp.setter
    def duty_cycle_sp(self, value):
        self.write('duty_cycle_sp', value)

    @property
    def is_holding(self):
        return self.STATE_HOLDING in self.state

    @property
    def is_overloaded(self):
        return self.STATE_OVERLOADED in self.state

    @property
    def is_ramping(self):
        return self.STATE_RAMPING in self.state

    @property
    def is_running(self):
        return self.STATE_RUNNING in self.state

    @property
    def is_stalled(self):
        return self.STATE_STALLED in self.state

    @property
    def polarity(self):
        return self.readStr('polarity')

    @polarity.setter
    def polarity(self, value):
        self.write('polarity', value)

    @property
    def position(self):
        return self.readInt('position')

    @position.setter
    def position(self, value):
        self.write('position', value)

    @property
    def position_d(self):
        return self.readInt('hold_pid/Kd')

    @position_d.setter
    def position_d(self, value):
        self.write('hold_pid/Kd', value)

    @property
    def position_i(self):
        return self.readInt('hold_pid/Ki')

    @position_i.setter
    def position_i(self, value):
        self.write('hold_pid/Ki', value)

    @property
    def position_p(self):
        return self.readInt('hold_pid/Kp')

    @position_p.setter
    def position_p(self, value):
        self.write('hold_pid/Kp', value)

    @property
    def position_sp(self):
        return self.readInt('position_sp')

    @position_sp.setter
    def position_sp(self, value):
        self.write('position_sp', value)

    @property
    def ramp_down_sp(self):
        return self.readInt('ramp_down_sp')

    @ramp_down_sp.setter
    def ramp_down_sp(self, value):
        self.write('ramp_down_sp', value)

    @property
    def ramp_up_sp(self):
        return self.readInt('ramp_up_sp')

    @ramp_up_sp.setter
    def ramp_up_sp(self, value):
        self.write('ramp_up_sp', value)

    @property
    def speed(self):
        return self.readInt('speed')

    @property
    def speed_d(self):
        return self.readInt('speed_pid/Kd')

    @speed_d.setter
    def speed_d(self, value):
        self.write('speed_pid/Kd', value)

    @property
    def speed_i(self):
        return self.readInt('speed_pid/Ki')

    @speed_i.setter
    def speed_i(self, value):
        self.write('speed_pid/Ki', value)

    @property
    def speed_p(self):
        return self.readInt('speed_pid/Kp')

    @speed_p.setter
    def speed_p(self, value):
        self.write('speed_pid/Kp', value)

    @property
    def speed_sp(self):
        return self.readInt('speed_sp')

    @speed_sp.setter
    def speed_sp(self, value):
        self.write('speed_sp', self.speed_sp_table[value])

    @property
    def state(self):
        return self.readStr('state')

    @property
    def stop_action(self):
        """
        Reading returns the current stop action.
        Use stop_actions for the list of possible values.
        """
        return self.readStr('stop_action')

    @stop_action.setter
    def stop_action(self, value):
        self.write('stop_action', value)

    @property
    def time_sp(self):
        return self.readInt('time_sp')

    @time_sp.setter
    def time_sp(self, value):
        self.write('time_sp', value)

    def reset(self, **kwargs):
        """
        Resets the motor the default value. It will also stop the motor.
        """
        for k in kwargs:
            setattr(self, k, kwargs[k])
        self.write('command', 'reset\n')
    
    def run_direct(self, **kwargs):
        """
        Run the motor at the duty cycle specified by duty_cycle_sp.
        Unlike other run commands, changing duty_cycle_sp
        while running will take effect immediately.
        """
        for k in kwargs:
            setattr(self, k, kwargs[k])
        self.write('command', 'run-direct\n')
    
    def run_forever(self, **kwargs):
        """
        Run the motor until another command is sent.
        """
        for k in kwargs:
            setattr(self, k, kwargs[k])
        self.write('command', 'run-forever\n')

    def run_timed(self, **kwargs):
        """
        Run for the amount of time specified in time_sp.
        Then, stop the motor as specified by stop_action.
        """
        for k in kwargs:
            setattr(self, k, kwargs[k])
        self.write('command', 'run-timed\n')

    def run_to_abs_pos(self, **kwargs):
        """
        Run to the absolute position as specified by position_sp.
        Then, stop the motor as specified by stop_action.
        """
        for k in kwargs:
            setattr(self, k, kwargs[k])
        self.write('command', 'run-to-abs-pos\n')

    def run_to_rel_pos(self, **kwargs):
        """
        Run to the relative position as specified by position_sp.
        New position will be current position + position_sp
        When the new position is reached, the motor will stop, as specified
        by stop_action.
        """
        for k in kwargs:
            setattr(self, k, kwargs[k])
        self.write('command', 'run-to-rel-pos\n')

    def stop(self, **kwargs):
        """
        Stop any of the run commands before they are complete using the
        action specified by stop_action.
        """
        for k in kwargs:
            setattr(self, k, kwargs[k])
        self.write('command', 'stop\n')

    def wait(self, cond, timeout=None):
        """
        Blocks until ``cond(self.state)`` is ``True``.  The condition is
        checked when there is an I/O event related to the ``state`` attribute.
        Exits early when ``timeout`` (in milliseconds) is reached.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.   

        Valid flags for state attribute: running, ramping, holding,
        overloaded and stalled
        """
        poll = select.poll()
        poll.register(self._state_fd, select.POLLIN)

        while True:
            event = poll.poll(timeout)

            if len(event) == 0:
                return False
            
            if cond(self.state):
                return True

    def wait_until(self, s, timeout=None):
        """
        Blocks until ``s`` is in ``self.state``.  The condition is checked when
        there is an I/O event related to the ``state`` attribute.  Exits early
        when ``timeout`` (in milliseconds) is reached.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.

        Example::
            m.wait_until('stalled')
        """
        return self.wait(lambda state: s in state, timeout)

    def wait_until_not_moving(self, timeout=None):
        """
        Blocks until one of the following conditions are met:
        - ``running`` is not in ``self.state``
        - ``stalled`` is in ``self.state``
        - ``holding`` is in ``self.state``
        The condition is checked when there is an I/O event related to
        the ``state`` attribute.  Exits early when ``timeout`` (in
        milliseconds) is reached.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.

        Example::

            m.wait_until_not_moving()
        """
        return self.wait(lambda state: self.STATE_RUNNING not in state or self.STATE_STALLED in state, timeout)

    def wait_while(self, s, timeout=None):
        """
        Blocks until ``s`` is not in ``self.state``.  The condition is checked
        when there is an I/O event related to the ``state`` attribute.  Exits
        early when ``timeout`` (in milliseconds) is reached.

        Returns ``True`` if the condition is met, and ``False`` if the timeout
        is reached.

        Example::

            m.wait_while('running')
        """
        return self.wait(lambda state: s not in state, timeout)

    def _set_rel_position_degrees_and_speed_sp(self, degrees, speed):
        degrees = degrees if speed >= 0 else -degrees
        speed = abs(speed)

        position_delta = int(round((degrees * self.count_per_rot) / 360))
        speed_sp = int(round(speed))

        self.position_sp = position_delta
        self.speed_sp = speed_sp

    def on_for_rotations(self, speed, rotations, brake=True, block=True):
        """
        Rotate the motor at ``speed`` for ``rotations``

        ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
        object, enabling use of other units.
        """
        if not isinstance(speed, SpeedValue):
            if -100 <= speed <= 100:
                speed = SpeedPercent(speed)
                speed_sp = speed.to_native_units(self)
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            speed_sp = int(round(speed.to_native_units(self)))

        self._set_rel_position_degrees_and_speed_sp(rotations * 360, speed_sp)

        if brake:
            self.stop_action = self.STOP_ACTION_HOLD
        else:
            self.stop_action = self.STOP_ACTION_COAST

        self.run_to_rel_pos()

        if block:
            self.wait_until('running', timeout=Motor.WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()
  
    def on_for_degrees(self, speed, degrees, brake=True, block=True):
        """
        Rotate the motor at ``speed`` for ``degrees``

        ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
        object, enabling use of other units.
        """
        if not isinstance(speed, SpeedValue):
            if -100 <= speed <= 100:
                speed = SpeedPercent(speed)
                speed_sp = speed.to_native_units(self)
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            speed_sp = int(round(speed.to_native_units(self)))

        self._set_rel_position_degrees_and_speed_sp(degrees, speed_sp)

        if brake:
            self.stop_action = self.STOP_ACTION_HOLD
        else:
            self.stop_action = self.STOP_ACTION_COAST

        self.run_to_rel_pos()

        if block:
            self.wait_until('running', timeout=Motor.WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()

    def on_to_position(self, speed, position, brake=True, block=True):
        """
        Rotate the motor at ``speed`` to ``position``

        ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
        object, enabling use of other units.
        """
        if not isinstance(speed, SpeedValue):
            if -100 <= speed <= 100:
                speed = SpeedPercent(speed)
                speed_sp = speed.to_native_units(self)
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            speed_sp = int(round(speed.to_native_units(self)))

        self.speed_sp = int(round(speed_sp))
        self.position_sp = position

        if brake:
            self.stop_action = self.STOP_ACTION_HOLD
        else:
            self.stop_action = self.STOP_ACTION_COAST

        self.run_to_abs_pos()

        if block:
            self.wait_until('running', timeout=Motor.WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()

    def on_for_seconds(self, speed, seconds, brake=True, block=True):
        """
        Rotate the motor at ``speed`` for ``seconds``

        ``speed`` can be a percentage or a :class:`ev3dev2.motor.SpeedValue`
        object, enabling use of other units.
        """
        if seconds < 0:
            raise ValueError("Seconds is negative.")

        if not isinstance(speed, SpeedValue):
            if -100 <= speed <= 100:
                speed = SpeedPercent(speed)
                speed_sp = speed.to_native_units(self)
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            speed_sp = int(round(speed.to_native_units(self)))

        self.speed_sp = int(round(speed_sp))
        self.time_sp = int(seconds * 1000)

        if brake:
            self.stop_action = self.STOP_ACTION_HOLD
        else:
            self.stop_action = self.STOP_ACTION_COAST

        self.run_timed()

        if block:
            self.wait_until('running', timeout=Motor.WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()

    def on(self, speed, brake=True, block=False):
        if not isinstance(speed, SpeedValue):
            if -100 <= speed <= 100:
                speed = SpeedPercent(speed)
                speed_sp = speed.to_native_units(self)
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            speed_sp = int(round(speed.to_native_units(self)))

        self.speed_sp = int(round(speed_sp))
    
        if brake:
            self.stop_action = self.STOP_ACTION_HOLD
        else:
            self.stop_action = self.STOP_ACTION_COAST

        self.run_forever()

        if block:
            self.wait_until('running', timeout=Motor.WAIT_RUNNING_TIMEOUT)
            self.wait_until_not_moving()
    
    def off(self, brake=True):
        if brake:
            self.stop_action = self.STOP_ACTION_HOLD
        else:
            self.stop_action = self.STOP_ACTION_COAST        
        self.stop()

class LargeMotor(Motor):
    _DRIVER_NAME = 'lego-ev3-l-motor'

class MediumMotor(Motor):
    _DRIVER_NAME = 'lego-ev3-m-motor'

#####################################################################################

class MotorSet:

    def __init__(self, motor_specs, desc=None):
        self.motors = {}
        for motor_port in sorted(motor_specs.keys()):
            motor_class = motor_specs[motor_port]
            self.motors[motor_port] = motor_class(motor_port)
            self.motors[motor_port].reset()
        self.desc = desc
        
    def __str__(self):
        if self.desc:
            return self.desc
        else:
            return self.__class__.__name__

    def reset(self, **kwargs):
        for motor in self.motors.values():
            for k in kwargs:
                setattr(self, k, kwargs[k])
            motor.write('command', 'reset\n')
  
    def run_forever(self, **kwargs):
        for motor in self.motors.values():
            for k in kwargs:
                setattr(motor, k, kwargs[k])
            motor.write('command','run-forever\n')

    def run_to_abs_pos(self, **kwargs):
        for motor in self.motors.values():
            for k in kwargs:
                setattr(motor, k, kwargs[k])
            motor.write('command','run-to-abs-pos\n')

    def run_to_rel_pos(self, **kwargs):
        for motor in self.motors.values():
            for k in kwargs:
                setattr(motor, k, kwargs[k])
            motor.write('command','run-to-rel-pos\n')

    def run_timed(self, **kwargs):
        for motor in self.motors.values():
            for k in kwargs:
                setattr(motor, k, kwargs[k])
            motor.write('command','run-timed\n')

    def run_direct(self, **kwargs):
        for motor in self.motors.values():
            for k in kwargs:
                setattr(motor, k, kwargs[k])
            motor.write('command','run-direct\n')

    def off(self, motors=None, brake=True):
        motors = motors if motors is not None else self.motors.values()
        
        for motor in motors:
            motor.stop_action = motor.STOP_ACTION_HOLD if brake else motor.STOP_ACTION_COAST
        
        for motor in motors:
            motor.stop()
    
    def stop(self, motors=None, brake=True):
        self.off(motors, brake)

    @property
    def is_running(self, motors=None, state=Motor.STATE_RUNNING):
        motors = motors if motors is not None else self.motors.values()
        for motor in motors:
            if state not in motor.state:
                return False
        return True
  
    @property
    def is_ramping(self, motors=None, state=Motor.STATE_RAMPING):
        motors = motors if motors is not None else self.motors.values()
        for motor in motors:
            if state not in motor.state:
                return False
        return True
  
    @property
    def is_holding(self, motors=None, state=Motor.STATE_HOLDING):
        motors = motors if motors is not None else self.motors.values()
        for motor in motors:
            if state not in motor.state:
                return False
        return True
  
    @property
    def is_overloaded(self, motors=None, state=Motor.STATE_OVERLOADED):
        motors = motors if motors is not None else self.motors.values()
        for motor in motors:
            if state not in motor.state:
                return False
        return True
  
    @property
    def is_stalled(self, motors=None, state=Motor.STATE_STALLED):
        motors = motors if motors is not None else self.motors.values()
        for motor in motors:
            if state not in motor.state:
                return False
        return True
  
    def wait(self, cond, timeout=None, motors=None):
        motors = motors if motors is not None else self.motors.values()

        for motor in motors:
            motor.wait(cond, timeout)

    def wait_until_not_moving(self, timeout=None, motors=None):
        motors = motors if motors is not None else self.motors.values()

        for motor in motors:
            motor.wait_until_not_moving(timeout)

    def wait_until(self, s, timeout=None, motors=None):
        motors = motors if motors is not None else self.motors.values()

        for motor in motors:
            motor.wait_until(s, timeout)

    def wait_while(self, s, timeout=None, motors=None):
        motors = motors if motors is not None else self.motors.values()

        for motor in motors:
            motor.wait_while(s, timeout)
  
    def _block(self):
        self.wait_until('running', timeout=Motor.WAIT_RUNNING_TIMEOUT)
        self.wait_until_not_moving()

#####################################################################################

class MoveTank(MotorSet):

    def __init__(self, left_motor_port, right_motor_port, desc=None, motor_class=LargeMotor):
        motor_specs = {
            left_motor_port : motor_class,
            right_motor_port : motor_class,
        }
        MotorSet.__init__(self, motor_specs, desc)
        self.left_motor = self.motors[left_motor_port]
        self.right_motor = self.motors[right_motor_port]
        self.max_speed = self.left_motor.max_speed
  
    def on_for_degrees(self, left_speed, right_speed, degrees, brake=True, block=True):
        """
        Rotate the motors at 'left_speed' and 'right_speed' for 'degrees'.
        Speeds can be percentages or any SpeedValue implementation.

        If the left speed is not equal to the right speed (i.e., the robot will
        turn), the motor on the outside of the turn will rotate for the full
        ``degrees`` while the motor on the inside will have its requested
        distance calculated according to the expected turn.
        """
        if not isinstance(left_speed, SpeedValue):
            if -100 <= left_speed <= 100:
                left_speed_obj = SpeedPercent(left_speed)
                left_speed_var = int(round(left_speed_obj.to_native_units(self.left_motor)))
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            left_speed_var = int(round(left_speed.to_native_units(self.left_motor)))
  
        if not isinstance(right_speed, SpeedValue):
            if -100 <= right_speed <= 100:
                right_speed_obj = SpeedPercent(right_speed)
                right_speed_var = int(round(right_speed_obj.to_native_units(self.right_motor)))
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            right_speed_var = int(round(right_speed.to_native_units(self.right_motor)))

        if degrees == 0 or (left_speed_var == 0 and right_speed_var == 0):
            left_degrees = 0
            right_degrees = 0
        elif abs(left_speed_var) > abs(right_speed_var):
            left_degrees = degrees
            right_degrees = abs(right_speed_var / left_speed_var) * degrees
        else:
            left_degrees = abs(left_speed_var / right_speed_var) * degrees
            right_degrees = degrees
    
        left_degrees_in = round((left_degrees * self.left_motor.count_per_rot) / 360)
        right_degrees_in = round((right_degrees * self.right_motor.count_per_rot) / 360)

        self.left_motor.position_sp = left_degrees_in if left_speed_var >= 0 else -left_degrees_in
        self.left_motor.speed_sp = abs(left_speed_var)
        self.right_motor.position_sp = right_degrees_in if right_speed_var >= 0 else -right_degrees_in
        self.right_motor.speed_sp = abs(right_speed_var)

        self.left_motor.stop_action = self.left_motor.STOP_ACTION_HOLD if brake else self.left_motor.STOP_ACTION_COAST
        self.right_motor.stop_action = self.right_motor.STOP_ACTION_HOLD if brake else self.right_motor.STOP_ACTION_COAST

        self.left_motor.run_to_rel_pos()
        self.right_motor.run_to_rel_pos()

        if block:
            self._block()
  
    def on_for_rotations(self, left_speed, right_speed, rotations, brake=True, block=True):
        """
        Rotate the motors at 'left_speed' and 'right_speed' for 'rotations'.
        Speeds can be percentages or any SpeedValue implementation.

        If the left speed is not equal to the right speed (i.e., the robot will
        turn), the motor on the outside of the turn will rotate for the full
        ``rotations`` while the motor on the inside will have its requested
        distance calculated according to the expected turn.
        """    
        MoveTank.on_for_degrees(self, left_speed, right_speed, rotations * 360, brake, block)

    def on_for_seconds(self, left_speed, right_speed, seconds, brake=True, block=True):
        """
        Rotate the motors at 'left_speed & right_speed' for 'seconds'.
        Speeds can be percentages or any SpeedValue implementation.
        """
        if seconds < 0:
            raise ValueError("Seconds is negative.")

        if not isinstance(left_speed, SpeedValue):
            if -100 <= left_speed <= 100:
                left_speed_obj = SpeedPercent(left_speed)
                left_speed_var = int(round(left_speed_obj.to_native_units(self.left_motor)))
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            left_speed_var = int(round(left_speed.to_native_units(self.left_motor)))
    
        if not isinstance(right_speed, SpeedValue):
            if -100 <= right_speed <= 100:
                right_speed_obj = SpeedPercent(right_speed)
                right_speed_var = int(round(right_speed_obj.to_native_units(self.right_motor)))
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            right_speed_var = int(round(right_speed.to_native_units(self.right_motor)))

        self.left_motor.speed_sp = left_speed_var
        self.left_motor.time_sp = seconds * 1000
        self.right_motor.speed_sp = right_speed_var
        self.right_motor.time_sp = seconds * 1000

        self.left_motor.stop_action = self.left_motor.STOP_ACTION_HOLD if brake else self.left_motor.STOP_ACTION_COAST
        self.right_motor.stop_action = self.right_motor.STOP_ACTION_HOLD if brake else self.right_motor.STOP_ACTION_COAST
        
        self.left_motor.run_timed()
        self.right_motor.run_timed()

        if block:
            self._block()    

    def on(self, left_speed, right_speed):
        """
        Start rotating the motors according to ``left_speed`` and ``right_speed`` forever.
        Speeds can be percentages or any SpeedValue implementation.

        """
        if not isinstance(left_speed, SpeedValue):
            if -100 <= left_speed <= 100:
                left_speed_obj = SpeedPercent(left_speed)
                left_speed_var = int(round(left_speed_obj.to_native_units(self.left_motor)))
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            left_speed_var = int(round(left_speed.to_native_units(self.left_motor)))
  
        if not isinstance(right_speed, SpeedValue):
            if -100 <= right_speed <= 100:
                right_speed_obj = SpeedPercent(right_speed)
                right_speed_var = int(round(right_speed_obj.to_native_units(self.right_motor)))
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            right_speed_var = int(round(right_speed.to_native_units(self.right_motor)))

        self.left_motor.speed_sp = left_speed_var
        self.right_motor.speed_sp = right_speed_var
        self.left_motor.run_forever()
        self.right_motor.run_forever()

#####################################################################################

class MoveSteering(MoveTank):
  
    def get_speed_steering(self, steering, speed):
        if steering > 100 or steering < -100:
            raise ValueError("Invalid Steering Value. Between -100 and 100 (inclusive).")

        # Assumes left motor's speed stats are the same as the right motor's
        if not isinstance(speed, SpeedValue):
            if -100 <= speed <= 100:
                speed_obj = SpeedPercent(speed)
                speed_var = speed_obj.to_native_units(self.left_motor)
            else:
                raise Exception("Invalid Speed Percentage. Speed must be between -100 and 100)")
        else:
            speed_var = speed.to_native_units(self.left_motor)

        left_speed = speed_var
        right_speed = speed_var
        speed_factor = (50 - abs(float(steering))) / 50

        if steering >= 0:
            right_speed *= speed_factor
        else:
            left_speed *= speed_factor
    
        return(left_speed, right_speed)

    def on_for_rotations(self, steering, speed, rotations, brake=True, block=True):
        """
        Rotate the motors according to the provided ``steering``.

        The distance each motor will travel follows the rules of :meth:`MoveTank.on_for_rotations`.
        """
        (left_speed, right_speed) = self.get_speed_steering(steering, speed)
        MoveTank.on_for_rotations(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), rotations, brake, block)

    def on_for_degrees(self, steering, speed, degrees, brake=True, block=True):
        """
        Rotate the motors according to the provided ``steering``.

        The distance each motor will travel follows the rules of :meth:`MoveTank.on_for_degrees`.
        """
        (left_speed, right_speed) = self.get_speed_steering(steering, speed)
        MoveTank.on_for_degrees(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), degrees, brake, block)

    def on_for_seconds(self, steering, speed, seconds, brake=True, block=True):
        """
        Rotate the motors according to the provided ``steering`` for ``seconds``.
        """
        (left_speed, right_speed) = self.get_speed_steering(steering, speed)
        MoveTank.on_for_seconds(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed), seconds, brake, block)
  
    def on(self, steering, speed):
        """
        Start rotating the motors according to the provided ``steering`` and
        ``speed`` forever.
        """
        (left_speed, right_speed) = self.get_speed_steering(steering, speed)
        MoveTank.on(self, SpeedNativeUnits(left_speed), SpeedNativeUnits(right_speed))

#####################################################################################

class Led(Device):
    """ This is a reference to the two LED objects located on 
        the face of the EV3 device.  There is the right (starboard) and
        left (port) leds.  The colors can be changed as well as the brightness levels.
    """
    _DIRECTORY_BASE = "/sys/class/leds/"

    LEFT = "left"
    RIGHT = "right"
    LOCATIONS = (LEFT, RIGHT)

    GREEN = "green"
    RED = "red"
    COLOR_TYPES = (GREEN, RED)


    LEDS = {}
    LEDS['red_left'] = 'led0:red:brick-status'
    LEDS['red_right'] = 'led1:red:brick-status'
    LEDS['green_left'] = 'led0:green:brick-status'
    LEDS['green_right'] = 'led1:green:brick-status'

    def __init__(self, location, color_type):
        super().__init__()
        location = location.lower()
        color_type = color_type.lower()
        if location not in Led.LOCATIONS:
            raise Exception("Invalid Location for Led. (" + location + ") attempted but is not in " + str(Led.LOCATIONS)+ ".")
        if color_type not in Led.COLOR_TYPES:
            raise Exception("Invalid Color Type for Led. (" + color_type + ") attempted but is not in " + str(Led.COLOR_TYPES)+ ".")
            
        self.attribute = color_type + "_"+ location
        self._directory = self._DIRECTORY_BASE + Led.LEDS[self.attribute] +"/"


    @property
    def max_brightness(self):
        """
        Returns the maximum possible brightness value.
        """
        return self.readInt("max_brightness")

    @property
    def brightness(self):
        """
        Returns the current brightness value
        """
        return self.readInt("brightness")
    
    @brightness.setter
    def brightness(self, value):
        """
        Sets the current brightness value
        """
        self.write("brightness", value)

    @property
    def triggers(self):
        """
        Returns the current brightness value
        """
        # retrieve all the possible triggers, including active one
        value = self.readStr("trigger")
        # make a list of the triggers from the string
        triggers = value.split()
        # loop through and remove the annotation around active trigger
        for i in range(len(triggers)):
          trigger = triggers[i]
          if trigger[0] == '[' and trigger[-1] == ']':
            triggers[i] = trigger[1:-1]  
        # return list of all triggers, minus annotation for active     
        return triggers

    @property
    def trigger(self):
        """
        Returns the current brightness value
        """
        # get all triggers
        value =  self.readStr("trigger")
        # turn triggers into a list
        debug(value.split(' '))
        trigger = None
        # find the active one, strip annotation
        for str in value.split(' '):
            if str[0] == '[' and str[-1] == ']':
                trigger = str[1:-1]
        # return active trigger name
        return trigger

    @trigger.setter    
    def trigger(self, value):
        """
        Sets the current trigger value
        """
        self.write("trigger", value)

#####################################################################################

class Sound:

    def __init__(self):
        pass

    def beep(self, args='', wait=True):
        """
        (borrowed and modified from ev3dev2)

        Call beep command with the provided arguments (if any).
        See `beep man page`_ and google `linux beep music`_ for inspiration.
        
        :param string args: Any additional arguments to be passed to ``beep`` (see the `beep man page`_ for details)
        
        Example: 
        sound = Sound()
        sound.beep("-f 700 -l 1000", True)

        This example sets the pitch to 700 for 1000 milliseconds and waits for completion
        
        .. _`beep man page`: https://linux.die.net/man/1/beep
        .. _`linux beep music`: https://www.google.com/search?q=linux+beep+music
        """
        if(wait):
            os.system("/usr/bin/beep %s" % args)
        else:
            os.system('{} &'.format("/usr/bin/beep %s" % args))
