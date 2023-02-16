import time
import threading
import numpy as np
import json
import rplidar
import pyrealsense2 as rs
from typing import Tuple
from . import vex_serial, assembly

class Ports:
  PORT1  =  0
  PORT2  =  1
  PORT3  =  2
  PORT4  =  3
  PORT5  =  4
  PORT6  =  5
  PORT7  =  6
  PORT8  =  7
  PORT9  =  8
  PORT10 =  9
  PORT11 = 10
  PORT12 = 11
  PORT13 = 12
  PORT14 = 13
  PORT15 = 14
  PORT16 = 15
  PORT17 = 16
  PORT18 = 17
  PORT19 = 18
  PORT20 = 19

class State:
  def __init__(self, sensorValues, timestamp=None):
    self.sensors = sensorValues
    self.timestamp = timestamp if timestamp else time.time()
    self.values = {}

class CortexController(vex_serial.VexCortex): # just a wrapper really with state mgmt
  _entity = None

  def __init__(self, path=None, baud=115200, desc=None):
    super().__init__(path=path, baud=baud)
    self._state = State(self.sensors(), 0.0)

    if not CortexController._entity:
      CortexController._entity = self

    if desc:
      with open(desc, "r") as fp:
        self.description = json.load(fp)
        self.model = assembly.load(self.description)
    else:
      self.description = {}
      self.model = None

  def state(self) -> State:
    # make sure we are always up to date
    updateable = False
    if self._last_rx_timestamp:
      updateable = not self._state.timestamp or \
        self._state.timestamp < self.timestamp() # yes, we are calling "twice" (as shown below)

    if updateable:
      self._rx_timestamp_lock.acquire()
      state = State(self.sensors(), self._last_rx_timestamp)
      self._rx_timestamp_lock.release()
      self._state = state

    return self._state

  def obs(self) -> np.ndarray:
    return None

  def act(self, _: np.ndarray):
    pass

  def connect(self):
    self.start()

  def disconnect(self):
    self.close()
    self.join()

  def running(self):
    return self.is_alive()

class RPLidar:
  def __init__(self, path="/dev/ttyUSB0"):
    super().__init__()
    self.device = rplidar(path)
    self.full_scan = None
    self.scan_buf = []
    self._lock = threading.Lock()
    self._keep_running = True
    self._thread = threading.Thread(target=self.run)

  def run(self):
    for scan in self.device.iter_scans():
      if not self._keep_running:
        break
      for pt in scan:
        if len(self.scan_buf) == 0 or self.scan_buf[-1][1] - pt[1] > 270:
          self._lock.acquire()
          self.full_scan = np.array(self.scan_buf, dtype=np.double)
          self.scan_buf = []
          self._lock.release()
        self.scan_buf.append(pt)
      if not self._keep_running:
        break

  def stop(self):
    self._keep_running = False
    if self.device:
      self._thread.join()
      self._thread = None
      self.device.stop()
      self.device.stop_motor()
      self.device.disconnect()
      self.device = None

  def read(self, unit="inches") -> np.ndarray:
    """
    Get a full scan from the lidar

    Args:
        unit (str, optional): inches|meters. Defaults to 'inches'.
    
    Returns:
        np.ndarray: [(quality, angle, distance), ...]
    """
    scan = None
    self._lock.acquire()
    if self.full_scan:
      scan = self.full_scan
      self.full_scan = None
    self._lock.release()
    if unit == "inches":
      scan[:,2] /= 0.0254
    return scan

  def __del__(self):
    self.stop()

class RealsenseCamera:
  def __init__(self, width=640, height=360):
    self.width = width
    self.height = height
    self.pipeline = rs.pipeline()
    
    config = rs.config()
    config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
    profile = self.pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    self.depth_scale = depth_sensor.get_depth_scale()

  def read(self, unit='inches') -> Tuple[bool, np.ndarray, np.ndarray]:
    """
    Grab frames from the realsense camera

    Args:
        unit (str, optional): inches|meters|z16. Defaults to 'inches'.

    Returns:
        Tuple[bool, np.ndarray, np.ndarray]: status, color image, depth image
    """
    if self.pipeline:
      try:
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        if unit == "inches":
          depth_image *= self.depth_scale / 0.0254
        elif unit == "meters":
          depth_image *= self.depth_scale
        return True, color_image, depth_image

      except:
        if self.pipeline:
          self.pipeline.stop()
          self.pipeline = None

    return False, None, None

  def __del__(self):
    if self.pipeline:
      self.pipeline.stop()
      self.pipeline = None