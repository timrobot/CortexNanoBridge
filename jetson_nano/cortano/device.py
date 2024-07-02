import threading
import signal
import ctypes
import numpy as np
import json
import multiprocessing
import os
import time
import pyrealsense2 as rs
from typing import Tuple
import cv2
from vex_serial import VexCortex
import logging

logging.basicConfig(filename="/var/log/cortano.log",
                    filemode='a',
                    format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                    datefmt='%H:%M:%S',
                    level=logging.DEBUG)

logging.info("Starting device log...")

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
  if VexCortex._entity:
    VexCortex._entity._keep_running.value = False
signal.signal(signal.SIGINT, handle_signal)

class RealsenseCamera:
  def __init__(self, width=640, height=360, autostart=True):
    """
    Initialize a realsense camera on this device
    """
    self.width = width
    self.height = height
    self.shape = (height, width)

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
    self.pipeline = None
    self.depth_scale = 0.001
    self.align = None
    if autostart:
      self.open()

    # just to make sure camera is still working
    self.last_frame_received = 0.0
    self.last_frame_reset = 0.5 # 500ms reset camera if nothing received

  def open(self):
    try:
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

    except Exception as e:
      try:
        self.pipeline.stop()
      except:
        pass
      self.pipeline = None

  def capture(self) -> Tuple[bool, np.ndarray, np.ndarray]:
    """
    Grab frames from the realsense camera

    Args:
        unit (str, optional): inches|meters|raw. Defaults to 'inches'.

    Returns:
        Tuple[bool, np.ndarray, np.ndarray]: status, color image, depth image
    """
    if self.pipeline is None:
      self.open()

    if self.pipeline is not None:
      try:
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        return True, color_image, depth_image

      except:
        logging.warning("Pipeline does not exist!")
        if self.pipeline:
          self.pipeline.stop()
          self.pipeline = None

    return False, None, None

  def __del__(self):
    self.close()

  def close(self):
    if self.pipeline:
      self.pipeline.stop()
      self.pipeline = None

  def read(self, scale=False): # will also store in buffer to be read
    ret, color, depth = self.capture()
    if not ret:
      logging.warning("Could not access proper frames")
      if time.time() - self.last_frame_received > self.last_frame_reset:
        logging.warning("Resetting pipeline since 0.5s passed")
        self.close()
      return np.zeros((self.height, self.width, 3), dtype=np.uint8), \
              np.zeros((self.height, self.width), dtype=np.float32)
    
    self.last_frame_received = time.time()
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

  def connected(self):
    return self.pipeline is not None
  
def getNextWebcamPath():
  for device_path in os.listdir('/sys/class/video4linux/'):
    if os.path.exists('/sys/class/video4linux/' + device_path + '/name'):
      with open('/sys/class/video4linux/' + device_path + '/name', 'r') as fp:
        device_name = fp.read()
      if 'realsense' not in device_name.lower():
        _path = '/dev/' + device_path
        cap = cv2.VideoCapture(_path)
        try:
          cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
          cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
          cap.set(cv2.CAP_PROP_FPS, 30)
          ret, frame = cap.read()
          ret, frame = cap.read() # try twice just to make sure
          if ret and frame is not None and len(frame.shape) == 3 and tuple(frame.shape) == (360, 640, 3):
            cap.release()
            return _path
          else:
            cap.release()
        except Exception as e:
          print(e)
          cap.release()
  return None