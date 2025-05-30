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
import logging
from .vex_serial import VexCortex

logging.basicConfig(filename="cortano.log",
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
  def __init__(self, autostart=True, postprocessing=True):
    """
    Initialize a realsense camera on this device
    """
    self.width = 640
    self.height = 360
    self.shape = (self.height, self.width)
    
    # used for USB 2.1 as found on the jetson nano
    self.fx = 460.92495728
    self.fy = 460.85058594
    self.cx = 315.10949707
    self.cy = 176.72598267
    self.hfov = 2 * np.arctan2(self.width  / 2, self.fx) # radians
    self.vfov = 2 * np.arctan2(self.height / 2, self.fy) # radians

    self.pipeline = None
    self.depth_scale = 0.001
    self.align = None
    if autostart:
      self.open()

    # just to make sure camera is still working
    self.last_frame_received = 0.0
    self.last_frame_reset = 0.5 # 500ms reset camera if nothing received
    self.postprocessing = postprocessing

    self.depth_to_disparity = rs.disparity_transform(True)
    self.disparity_to_depth = rs.disparity_transform(False)

    self.decimation = rs.decimation_filter()
    self.decimation_filter_magnitude = 1 if not postprocessing else 2
    self.decimation.set_option(rs.option.filter_magnitude, self.decimation_filter_magnitude)

    self.spatial = rs.spatial_filter()

    # do not use temporal filter for now since we are moving around, and temporal filter is best for static scenes
    self.temporal = rs.temporal_filter()
    self.temporal_frames = []

    self.hole_filling = rs.hole_filling_filter()

  def open(self):
    try:
      self.pipeline = rs.pipeline()
      
      config = rs.config()
      # decimation is only applied to depth stream
      depth_width = self.width if not self.postprocessing else self.width * self.decimation_filter_magnitude
      depth_height = self.height if not self.postprocessing else self.height * self.decimation_filter_magnitude
      config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
      config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, 30)
      self.profile = self.pipeline.start(config)
      
      # intel realsense on jetson nano sometimes get misdetected as 2.1 even though it has 3.2 USB
      self.color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
      self.color_intrinsics = self.color_profile.get_intrinsics()

      # Get the depth stream's video profile
      self.depth_profile = self.profile.get_stream(rs.stream.depth).as_video_stream_profile()
      self.depth_intrinsics = self.depth_profile.get_intrinsics()
      self.fx = self.depth_intrinsics.fx / self.decimation_filter_magnitude
      self.fy = self.depth_intrinsics.fy / self.decimation_filter_magnitude
      self.cx = self.ppx = self.depth_intrinsics.ppx / self.decimation_filter_magnitude
      self.cy = self.ppy = self.depth_intrinsics.ppy / self.decimation_filter_magnitude
      self.hfov = 2 * np.arctan2(self.width  / 2, self.fx) # radians
      self.vfov = 2 * np.arctan2(self.height / 2, self.fy) # radians

      depth_sensor = self.profile.get_device().first_depth_sensor()
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

        if color_frame is None or depth_frame is None:
          return False, None, None

        if self.postprocessing:
          depth_frame = self.decimation.process(depth_frame)
          depth_frame = self.spatial.process(depth_frame)
          # depth_frame = self.temporal.process(depth_frame)
          depth_frame = self.hole_filling.process(depth_frame)

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

  def read(self, scale=True) -> Tuple[np.ndarray, np.ndarray]: # will also store in buffer to be read
    """
    Read a frame from the realsense camera

    Args:
        scale (bool, optional): Convert uint16 depth to float32 meters. Defaults to True.

    Returns:
        Tuple[np.ndarray, np.ndarray]: color image, depth image
    """
    ret, color, depth = self.capture()
    if not ret:
      logging.warning("Could not access proper frames")
      if time.time() - self.last_frame_received > self.last_frame_reset:
        logging.warning("Resetting pipeline since 0.5s passed")
        self.close()
      return np.zeros((self.height, self.width, 3), dtype=np.uint8), \
             np.zeros((self.height, self.width), dtype=np.uint16)
    
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