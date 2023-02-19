import threading
import signal
import ctypes
import numpy as np
import json
import multiprocessing
import rplidar
import pyrealsense2 as rs
from typing import Tuple
from . import vex_serial, assembly

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

class Robot(vex_serial.VexCortex): # just a wrapper really with state mgmt
  _entity = None

  def __init__(self, path=None, baud=115200, model=None):
    super().__init__(path=path, baud=baud)
    if not Robot._entity:
      Robot._entity = self

    if model:
      with open(model, "r") as fp:
        self.description = json.load(fp)
        self.model = assembly.load(self.description)
    else:
      self.description = {}
      self.model = None

  def obs(self) -> np.ndarray:
    return None

  def act(self, _: np.ndarray):
    pass

_kill_event = threading.Event()
def handle_signal(sig, frame):
  _kill_event.set()
  if Robot._entity:
    Robot._entity._keep_running.value = False
signal.signal(signal.SIGINT, handle_signal)

def _run_rplidar(path, full_scan, keep_running, lock):
  scan_buf = []
  device = rplidar.RPLidar(path)
  finish_config = False
  while not finish_config:
    try:
      device.get_info()
      device.get_health()
      device.clean_input()
      finish_config = True
    except Exception as e:
      print(e)

  while keep_running.value:
    for scan in device.iter_scans():
      if not keep_running.value:
        break
      for pt in scan:
        if len(scan_buf) == 0 or scan_buf[-1][1] - pt[1] > 270:
          lock.acquire()
          full_scan[:] = scan_buf
          scan_buf = []
          lock.release()
        scan_buf.append(pt)
      if not keep_running.value:
        break

  device.stop()
  device.stop_motor()
  device.disconnect()

class RPLidar:
  def __init__(self, path="/dev/ttyUSB0"):
    super().__init__()
    self._manager = multiprocessing.Manager()
    self.full_scan = self._manager.list()
    self._lock = multiprocessing.Lock()
    self._keep_running = multiprocessing.RawValue(ctypes.c_bool, True)
    self._worker = multiprocessing.Process(target=_run_rplidar,
      args=(path, self.full_scan, self._keep_running, self._lock))
    self._worker.start()

  def stop(self):
    self._keep_running.value = False
    if self._worker:
      self._worker.join(3)
      if self._worker.is_alive():
        self._worker.kill()
      self._worker = None

  def read(self, unit="inches") -> np.ndarray:
    """
    Get a full scan from the lidar

    Args:
        unit (str, optional): inches|meters|mm. Defaults to 'inches'.
    
    Returns:
        np.ndarray: [(quality, angle, distance), ...]
    """
    scan = None
    self._lock.acquire()
    if self.full_scan:
      scan = np.array(self.full_scan, dtype=np.double)
      self.full_scan[:] = []
    self._lock.release()
    if scan is not None:
      if unit == "inches":
        scan[:,2] /= 25.4
      elif unit == "meters":
        scan[:,2] *= 0.001

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
        unit (str, optional): inches|meters|raw. Defaults to 'inches'.

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
          depth_image = depth_image.astype(np.double) * self.depth_scale / 0.0254
        elif unit == "meters":
          depth_image = depth_image.astype(np.double) * self.depth_scale
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