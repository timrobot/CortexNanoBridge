import threading
import signal
import ctypes
import numpy as np
import json
import multiprocessing
import os
import pyrealsense2 as rs
from typing import Tuple
import cv2
from . import vex_serial

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

_kill_event = threading.Event()
def handle_signal(sig, frame):
  _kill_event.set()
  if vex_serial.VexCortex._entity:
    vex_serial.VexCortex._entity._keep_running.value = False
signal.signal(signal.SIGINT, handle_signal)

class RealsenseCamera:
  def __init__(self, width=640, height=360):
    self.width = width
    self.height = height
    self.shape = (height, width)

    self.pipeline = rs.pipeline()
    
    config = rs.config()
    config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
    profile = self.pipeline.start(config)
    # intel realsense on jetson nano sometimes get misdetected as 2.1 even though it has 3.2 USB
    # profile = self.pipeline.start()

    depth_sensor = profile.get_device().first_depth_sensor()
    self.depth_scale = depth_sensor.get_depth_scale()

    self.align = rs.align(rs.stream.color)

    # self.intrinsic_matrix = self.metadata.intrinsics.intrinsic_matrix
    # used for USB 2.1 as found on the jetson nano
    if height == 480:
      self.fx = 614.5665893554688
      self.fy = 614.4674682617188
      self.cx = 313.47930908203125
      self.cy = 235.6346435546875
    elif height == 360:
      self.fx = 460.92495728
      self.fy = 460.85058594
      self.cx = 315.10949707
      self.cy = 176.72598267

    self.hfov = np.degrees(np.arctan2(self.width  / 2, self.fx)) * 2
    self.vfov = np.degrees(np.arctan2(self.height / 2, self.fy)) * 2

  def capture(self) -> Tuple[bool, np.ndarray, np.ndarray]:
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
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
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

  def read(self, scale=False): # will also store in buffer to be read
    ret, color, depth = self.capture()
    if not ret:
      return np.zeros((self.height, self.width, 3), dtype=np.uint8), \
             np.zeros((self.height, self.width), dtype=np.float32)
    if scale:
      depth = depth.astype(np.float32) * self.depth_scale
    # color = np.ascontiguousarray(np.flip(color, axis=-1))

    return color, depth

  def depth2rgb(self, depth):
    if depth.dtype == np.uint16:
      return cv2.applyColorMap(np.sqrt(depth).astype(np.uint8), cv2.COLORMAP_JET)
    else:
      return cv2.applyColorMap(np.floor(np.sqrt(depth / self.depth_scale)).astype(np.uint8), cv2.COLORMAP_JET)

  def view(self):
    color, depth = self.read()
    combined = np.hstack((color, self.depth2rgb(depth)))
    cv2.imshow("combined", combined)
    cv2.waitKey(1)
