import threading
import sys
import glob
import serial
import signal
import time
from multiprocessing import RawArray
from ctypes import c_float

class ProtectedZeroList:
  def __init__(self, length, writeable=True, permissive=None):
    self._data = RawArray(c_float, length)
    self._writeable = writeable
    self._permissive = permissive
    self._lock = threading.Lock()

  def __len__(self):
    return len(self._data)

  def __setitem__(self, idx, val):
    if not self._writeable:
      raise PermissionError("Cannot write to immutable list")
    if len(self._data) <= idx:
      raise ValueError("Index exceeds size of list")
    self._lock.acquire()
    self._data[idx] = val
    self._lock.release()

  def copy(self, arr, src=None):
    if not src or src is not self._permissive:
      if not self._writeable:
        raise PermissionError("Cannot write to immutable list")
      if len(arr) != len(self._data):
        raise ValueError(f"Input list size does not match required list size of {len(self._data)}")
      self._lock.acquire()
      for i, v in enumerate(arr):
        self._data[i] = v
      self._lock.release()
    else:
      self._lock.acquire()
      self._data = [v for v in arr]
      self._lock.release()

  def __iter__(self):
    return iter(self._data)

  def __getitem__(self, idx):
    if len(self._data) <= idx:
      raise ValueError("Index exceeds size of list")
    self._lock.acquire()
    val = self._data[idx]
    self._lock.release()
    return val

  def clone(self):
    _new_copy = [0] * len(self._data)
    self._lock.acquire()
    for i in range(len(self._data)):
      _new_copy[i] = self._data[i]
    self._lock.release()
    return _new_copy

  def __str__(self):
    return str([x for x in self._data])

RXMAX = 200
CMD_CONTROL_MOTOR_VALUES = 'M'
CMD_STATUS_SENSOR_VALUES = 'S'
CMD_STATUS_DEBUG         = 'I'
MLIMIT = 127

_kill_event = threading.Event()
def handle_signal(sig, frame):
  _kill_event.set()
signal.signal(signal.SIGINT, handle_signal)

class VexCortex(threading.Thread):
  def __init__(self, path=None, baud=115200):
    super().__init__()

    self.path = path
    self.baud = baud
    self._connection = None
    self._enabled = True #False

    self._read_buf = ""
    self._write_buf = ""

    self._rxbuf = ""
    self._txbuf = ""

    self._sensor_values = ProtectedZeroList(20, False, self)
    self._last_rx_timestamp = None
    self._rx_timestamp_lock = threading.Lock()
    self._motor_values = ProtectedZeroList(10)
    self._last_tx_timestamp = None

  def _autofind_path(self): # todo: do a more dynamic path finder using prefix
    # https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python?msclkid=bafb28c0ceb211ec97c565cfa73ea467
    if sys.platform.startswith('win'):
      paths = ["COM%s" % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux'):
      paths = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    else:
      raise EnvironmentError("Unsupported platform")

    if len(paths) == 0:
      raise EnvironmentError("Cannot find suitable port")

    for path in paths:
      try:
        self._connection = serial.Serial(path, self.baud)
        self.path = path
        break # once it has found one, we are good
      except (OSError, serial.SerialException):
        self._connection = None
        self.path = None

  def _receive_data(self):
    while (c := self._connection.read()):
      if c == b'\n':
        self._read_buf = self._rxbuf
        self._rxbuf = ""
        if self._read_buf[0] == '[' and self._read_buf[-1] == ']':
          return self._read_buf
        else:
          break
      else:
        try:
          if c == b'[':
            self._rxbuf = ""
          self._rxbuf += c.decode()
        except:
          pass
      if len(self._rxbuf) > RXMAX:
        self._rxbuf = ""
        break
    return None

  def _decode_message(self, msg):
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

      ptr = 4
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
      print("Error: could not decode message, incorrect hexstring read")
      return None

  def _send_message(self, vals):
    motor_correction = lambda x: \
      (-MLIMIT if x < -MLIMIT else (MLIMIT if x > MLIMIT else x)) + MLIMIT
    valstr = "".join(["%02x" % motor_correction(int(v)) for v in vals])
    length = len(valstr) + 7
    msg = "[M%02x%s]" % (length, valstr)
    chk_sum = 0
    for c in msg:
      chk_sum ^= ord(c)
    msg = msg[:-1] + ("%02x" % chk_sum) + "]\n"
    self._connection.write(msg.encode())

  def enabled(self):
    return not self._enabled

  def run(self, argpasser=None):
    if not self.path:
      self._autofind_path()
      if not self.path:
        raise EnvironmentError(f"Could not find any path")
    else:
      try:
        self._connection = serial.Serial(self.path, self.baud)
      except (OSError, serial.SerialException):
        raise EnvironmentError(f"Could not find specified path: {self.path}")

    global _kill_event
    while not _kill_event.is_set():
      rx = self._receive_data()
      if rx:
        values = self._decode_message(rx)
        if values:
          self._rx_timestamp_lock.acquire()
          self._sensor_values.copy(values, self)
          self._last_rx_timestamp = time.time()
          self._rx_timestamp_lock.release()

      # outgoing 50hz, incoming 100hz
      t = time.time()
      if self._last_tx_timestamp is None or t - self._last_tx_timestamp > 0.02:
        self._last_tx_timestamp = t
        motor_values = self._motor_values.clone()
        if not self._enabled:
          for i in range(len(motor_values)):
            motor_values[i] = 0
        self._send_message(motor_values)

    self.clean()

  def timestamp(self):
    self._rx_timestamp_lock.acquire()
    timestamp = self._last_rx_timestamp
    self._rx_timestamp_lock.release()
    return timestamp

  def stop_motors(self):
    for i in range(10):
      self._motor_values[i] = 0
    self._send_message([0] * 10)

  def clean(self):
    if self._connection:
      for i in range(10):
        self.stop_motors()
        time.sleep(0.05)
      self._connection.close()
      self._connection = None
    self.path = None
    self._last_tx_timestamp = None
    self._sensor_values = []

  def stop_connection(self):
    signal.raise_signal(signal.SIGTERM)

  @property
  def motor(self):
    """Return the reference to the motor array

    Returns:
        ProtectedZeroList: reference to the motor values
    """
    return self._motor_values

  def motors(self):
    """Get the motor values from the microcontroller

    Returns:
        List[float]: a copy of the motor values
    """
    return self._motor_values.clone()

  @property
  def sensor(self):
    return self._sensor_values

  def sensors(self):
    return self._sensor_values.clone()