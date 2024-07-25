import sys
import glob
import serial
import time
from multiprocessing import Process, Array, RawValue, Value
from ctypes import c_double, c_bool, c_int
import logging
import socket

class IndexableArray:
  def __init__(self, length):
    self._data = Array(c_int, length)

  def __len__(self):
    return len(self._data)

  def __setitem__(self, idx, val):
    self._data.acquire()
    self._data[idx] = val
    self._data.release()

  def set(self, arr):
    self._data.acquire()
    self._data[:len(arr)] = arr
    self._data.release()

  def __iter__(self):
    return iter(self._data)

  def __getitem__(self, idx):
    if len(self._data) <= idx:
      raise ValueError("Index exceeds size of list")
    self._data.acquire()
    val = self._data[idx]
    self._data.release()
    return val

  def clone(self):
    self._data.acquire()
    _new_copy = self._data[:]
    self._data.release()
    return _new_copy

  def __str__(self):
    return str(self._data[:])

CMD_CONTROL_MOTOR_VALUES  = 'M'
CMD_STATUS_DEBUG          = 'I'
CMD_STATUS_SENSOR_VALUES  = 'S'
CMD_INFO_IP               = 'W'
MLIMIT = 127

def get_ipv4():
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  s.connect(("8.8.8.8", 80))
  addr = s.getsockname()[0]
  s.close()
  return addr

def _decode_message(msg):
  if len(msg) == 0: return None
  if msg[0] != '[' or msg[-1] != ']': return None

  if msg[1] not in [CMD_STATUS_SENSOR_VALUES]:#, CMD_STATUS_DEBUG]:
    return None

  sensor_values = []
  try:
    length = int(msg[2:4], base=16)
    if length != len(msg): return None

    chk_sum = int(msg[-3:-1], base=16)
    for c in msg[:-3]:
      chk_sum ^= ord(c)
    chk_sum ^= ord(msg[-1])
    if chk_sum != 0: return None

    battery_level = int(msg[4:8], base=16)
    sensor_values.append(battery_level) # the first value will always be the battery level

    ptr = 8
    while ptr < length - 3:
      _type = msg[ptr]
      if _type == 'w':
        nbytes = 1
      elif _type == 's':
        nbytes = 4
      elif _type == 'l':
        nbytes = 8
      else:
        nbytes = 1
      ptr += 1

      data = int(msg[ptr:ptr+nbytes], base=16)
      # detect negative values by looking at first bit in first byte
      if int(msg[ptr], base=16) & 0x8:
        max_byte_value = (1 << (nbytes << 2))
        data -= max_byte_value
      sensor_values.append(data)
      ptr += nbytes

    return sensor_values

  except ValueError:
    logging.error("Could not decode message, incorrect hexstring read")
    return None

def _receive_data(connection, rxbuf):
  in_waiting = connection.in_waiting
  if in_waiting == 0:
    return None, rxbuf
  buf = connection.read(in_waiting)
  msg = None
  if buf:
    rxbuf += buf.decode()
    end = rxbuf.find(']')
    if end != -1:
      start = rxbuf.find('[', 0, end)
      if start != -1 and '[' not in rxbuf[start+1:end]:
        msg = rxbuf[start:end+1]
      rxbuf = rxbuf[end+1:]
  return msg, rxbuf

def _send_message(connection, vals):
  motor_correction = lambda x: \
    (-MLIMIT if x < -MLIMIT else (MLIMIT if x > MLIMIT else x)) + 0x7F
  valstr = "".join(["%02x" % motor_correction(int(v)) for v in vals])
  length = len(valstr) + 7
  msg = "[M%02x%s]" % (length, valstr)
  chk_sum = 0
  for c in msg:
    chk_sum ^= ord(c)
  msg = msg[:-1] + ("%02x" % chk_sum) + "]\n"
  connection.write(msg.encode())
  connection.flush()

def _send_ip(connection, ip):
  vals = [int(x) for x in ip.split(".")]
  valstr = "".join(["%02x" % v for v in vals])
  msg = "[W%s]" % (valstr)
  connection.write(msg.encode())
  connection.flush()

def _serial_worker(path, baud, motors, sensors, nsensors, enabled, readtime, keep_running, connected, _id=0, _rxch=None):
  rxbuf = ""
  last_tx_time = time.time()
  last_ip_time = time.time()

  while keep_running.value:
    if not connected.value:
      try:
        connection = serial.Serial(path, baud, timeout=1)
        connection.reset_output_buffer()
        connection.reset_input_buffer()
        connected.value = True
      except Exception as e:
        connection.close()
        connected.value = False
        time.sleep(0.01)

    if not connected.value: continue
    if _rxch is None or _rxch.value == -1 or _rxch.value == _id:
      try:
        rx, rxbuf = _receive_data(connection, rxbuf)
        if rx:
          if _rxch: _rxch.value = _id
          values = _decode_message(rx)
          if values:
            sensors._data.acquire()
            nsensors.value = len(values)
            sensors._data[:len(values)] = values
            sensors._data.release()
            readtime.acquire()
            readtime.value = time.time()
            readtime.release()
      except serial.SerialTimeoutException as e:
        continue
      except serial.SerialException as e:
        #There is no new data from serial port
        pass
      except TypeError as e:
        #Disconnect of USB->UART occured
        connection.close()
        connected.value = False

    if not connected.value or (_rxch and _rxch.value == _id): continue
    # outgoing 50hz
    t = time.time()
    if t - last_tx_time >= 0.02:
      last_tx_time = t
      values = [0] * len(motors)
      if enabled.value:
        values = motors.clone()
      try:
        _send_message(connection, values)
        if _rxch and t - last_ip_time >= 1.0: # only if v5
          last_ip_time = t
          _send_ip(connection, str(get_ipv4()))
      except serial.SerialTimeoutException as e:
        connection.close()
        connected.value = False

    time.sleep(0.005) # throttle to prevent CPU overload
  
  motors.set([0] * len(motors))
  for _ in range(10):
    _send_message(connection, [0] * len(motors))
    time.sleep(0.05)
  connection.close()

class VexMicrocontroller:
  _entity = None

  def __init__(self, path=None, baud=115200, motor_max=1):
    if VexMicrocontroller._entity is None:
      VexMicrocontroller._entity = self
    else:
      raise Exception("Already created VexMicrocontroller")
    
    self.baud = baud
    self.path = path

    self.motor_max = motor_max
    global MLIMIT
    MLIMIT = motor_max
    self._enabled = Value(c_bool, True)
    self._keep_running = RawValue(c_bool, True)

    self._sensor_values = IndexableArray(21) # include the battery
    self._num_sensors = Value(c_int, 1) # the first value will always be battery
    self._last_rx_time = Value(c_double, 0.0)
    self._motor_values = IndexableArray(10)

  def stop(self):
    self._keep_running.value = False

  def __del__(self):
    self.stop()

  def _get_potential_paths(self): # todo: do a more dynamic path finder using prefix
    # https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python?msclkid=bafb28c0ceb211ec97c565cfa73ea467
    if sys.platform.startswith('win'):
      paths = ["COM%s" % (i + 1) for i in range(128)]
    elif sys.platform.startswith('linux'):
      paths = glob.glob("/dev/ttyUSB*") #+ glob.glob("/dev/ttyACM*")
    else:
      raise EnvironmentError("Unsupported platform")

    if len(paths) == 0:
      raise EnvironmentError("Cannot find suitable port")

    for i in range(len(paths)):
      try:
        s = serial.Serial(paths[i], self.baud)
        s.close()
      except (OSError, serial.SerialException):
        paths[i] = None
    
    return [x for x in paths if x is not None]

  def enabled(self):
    return not self._enabled.value
  
  def running(self):
    return self._keep_running.value

  def timestamp(self):
    self._last_rx_time.acquire()
    timestamp = self._last_rx_time.value
    self._last_rx_time.release()
    return timestamp

  @property
  def motor(self):
    """Reference to the motor array

    Returns:
        IndexableArray: reference to the motor values
    """
    return self._motor_values

  def motors(self, motor_values):
    """Set motor values

    Args:
        motor_values (List[int]): motor values
    """
    self._motor_values.set(motor_values)

  def sensors(self):
    """Get the sensor values

    Returns:
        Tuple[List[int], int]: sensor values, battery_level
    """
    self._sensor_values._data.acquire()
    num_sensors = self._num_sensors.value
    sensor_values = self._sensor_values._data[1:num_sensors]
    battery_level = self._sensor_values._data[0]
    self._sensor_values._data.release()
    if battery_level > 100: battery_level / 1e3
    return sensor_values, battery_level
  
class VexCortex(VexMicrocontroller):
  def __init__(self, path=None, baud=115200):
    super().__init__(path, baud, motor_max=127)

    if self.path:
      try:
        s = serial.Serial(self.path, self.baud)
        s.close()
      except (OSError, serial.SerialException):
        raise EnvironmentError(f"Could not find specified path: {self.path}")
    else:
      paths = self._get_potential_paths()
      if paths:
        self.path = paths[0]
      else:
        raise EnvironmentError(f"Could not find any path")

    self._connected = RawValue(c_bool, False)
    self._worker = Process(target=_serial_worker, args=(
      self.path, self.baud, self._motor_values, self._sensor_values,
      self._num_sensors, self._enabled, self._last_rx_time, self._keep_running, self._connected))
    self._worker.start()

  def stop(self):
    super().stop()
    if self._worker:
      self._worker.join(3)
      if self._worker.is_alive():
        self._worker.kill()
      self._worker = None

class VexV5(VexMicrocontroller):
  def __init__(self, path1=None, path2=None, baud=115200):
    super().__init__([path1, path2], baud, motor_max=100)

    if path1 and path2:
      try:
        s = serial.Serial(path1, self.baud)
        s.close()
      except (OSError, serial.SerialException):
        raise EnvironmentError(f"Could not find specified path: {path1}")
      try:
        s = serial.Serial(path2, self.baud)
        s.close()
      except (OSError, serial.SerialException):
        raise EnvironmentError(f"Could not find specified path: {path2}")
    else:
      paths = self._get_potential_paths()
      if len(paths) >= 2:
        self.path1 = paths[0]
        self.path2 = paths[1]
      else:
        raise EnvironmentError(f"Could not find 2 paths")
      
    # each USB is a separate data channel - either rx or tx
    # we will need to wait on a read before we can assign the tx channel
    self.rx = Value(c_int, -1)

    self._connected1 = RawValue(c_bool, False)
    self._connected2 = RawValue(c_bool, False)
    self._worker1 = Process(target=_serial_worker, args=(
      self.path1, self.baud, self._motor_values, self._sensor_values,
      self._num_sensors, self._enabled, self._last_rx_time, self._keep_running, self._connected1, 0, self.rx))
    self._worker2 = Process(target=_serial_worker, args=(
      self.path2, self.baud, self._motor_values, self._sensor_values,
      self._num_sensors, self._enabled, self._last_rx_time, self._keep_running, self._connected2, 1, self.rx))
    self._worker1.start()
    self._worker2.start()

  def stop(self):
    super().stop()
    if self._worker1:
      self._worker1.join(3)
      if self._worker1.is_alive():
        self._worker1.kill()
      self._worker1 = None
    if self._worker2:
      self._worker2.join(3)
      if self._worker2.is_alive():
        self._worker2.kill()
      self._worker2 = None