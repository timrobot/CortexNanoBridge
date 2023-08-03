import cv2
import numpy as np

import socket
import pickle
import struct
import threading
import json
import time
import requests
import select
import signal
import sys

from multiprocessing import (
  RawValue
)
import ctypes

from . import rxtx

_frame_shape = (360, 640)
_frame = None
_frame_lock = threading.Lock()
_running = RawValue(ctypes.c_bool, False)
_connected = RawValue(ctypes.c_bool, False)
_color_encoding_parameters = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
_depth_encoding_parameters = [int(cv2.IMWRITE_PNG_COMPRESSION), 1]

def _stream_sender(host, port):
  """
  Streams data out
  """
  global _frame, _frame_lock, _frame_shape, _running, _connected
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.bind((host, port))
  sock.listen()
  _connected.value = is_connected = False
  conn = None

  while _running.value:
    if not is_connected:
      try:
        ready_to_read, ready_to_write, in_error = select.select(
          [sock, ], [], [], 1
        )
        for s in ready_to_read:
          if s is sock:
            conn, address = sock.accept()
            _connected.value = is_connected = True

      except Exception as e:
        print("Warning:", e)
        continue

    if not is_connected: continue

    _frame_lock.acquire()
    color, depth = _frame
    _frame_lock.release()

    result, color = cv2.imencode('.jpg', color, _color_encoding_parameters)
    result, depth = cv2.imencode('.png', depth, _depth_encoding_parameters)
    data = pickle.dumps((color, depth), 0)
    size = len(data)

    try:
        conn.sendall(struct.pack('>L', size) + data)
    except ConnectionResetError:
        _connected.value = is_connected = False
        conn.close()
    except ConnectionAbortedError:
        _connected.value = is_connected = False
        conn.close()
    except BrokenPipeError:
        _connected.value = is_connected = False
        conn.close()

  sock.close()

_host = "0.0.0.0"
_port = 9999

_stream_thread = None

def start(host, port=9999, frame_shape=(360, 640)):
  global _frame_shape, _frame, _host, _port, _running, _stream_thread
  _host = "0.0.0.0"
  _port = port
  _frame_shape = frame_shape
  _frame = (
     np.zeros((_frame_shape[0], _frame_shape[1], 3), np.uint8),
     np.zeros((_frame_shape[0], _frame_shape[1]), np.uint16)
  )

  if _running.value:
    print("Warning: stream is already running")
  else:
    _running.value = True
    rxtx.start_rxtx(_host, _port + 1)

    _stream_thread = threading.Thread(target=_stream_sender, args=(_host, _port))
    _stream_thread.start()

def stop():
  global _running
  if _running.value:
    _running.value = False
    rxtx.stop_rxtx()
    time.sleep(1)

def sig_handler(signum, frame):
  if signum == signal.SIGINT:
    stop()
    sys.exit()

signal.signal(signal.SIGINT, sig_handler)

def _encode_frame(color, depth):
  x = depth
  h, w = x.shape
  x1 = np.right_shift(np.bitwise_and(x, 0x0000ff00), 8).astype(np.uint8)
  x2 = np.bitwise_and(x, 0x000000ff).astype(np.uint8)
  frame = np.concatenate((x1.reshape((h, w, 1)), x1.reshape((h, w, 1)), x2.reshape((h, w, 1))), axis=-1)

  frame = np.concatenate((color, frame), axis=1)
  return frame

def set_frame(color: np.ndarray, depth: np.ndarray):
  global _frame_lock, _frame
  if color is None or depth is None: return
  # frame = _encode_frame(color, depth)
  _frame_lock.acquire()
  _frame = (color, depth)
  _frame_lock.release()

def write(sensor_values, voltage_level=None):
  """Send motor values to remote location

  Args:
      sensor_values (List[int]): sensor values
      voltage_level (int, optional): Voltage of the robot. Defaults to None.
  """
  sensor_values = [int(x) for x in sensor_values]
  rxtx._sensor_values.acquire()
  nsensors = rxtx._num_sensors.value = min(20, len(sensor_values))
  rxtx._sensor_values[:nsensors] = sensor_values
  rxtx._sensor_values.release()

def readtime():
  rxtx._last_rx_time.acquire()
  rxtime = rxtx._last_rx_time.value
  rxtx._last_rx_time.release()
  return rxtime

def read():
  """Return motor values that are read in

  Returns:
      List[int]: motor values
  """
  rxtx._motor_values.acquire()
  motor_values = rxtx._motor_values[:]
  rxtx._motor_values.release()
  rxtime = readtime()
  if (time.time() - rxtime) > 0.5: # timeout 0.5s before setting motors to 0
    motor_values = [0] * 10
  return motor_values

def check_alive():
  if rxtx.controlled_entity is None: return
  rxtime = readtime()
  if (time.time() - rxtime) > 0.5: # timeout 0.5s before setting motors to 0
    rxtx.controlled_entity.motors([0] * 10)

def control(robot):
  rxtx.set_passthrough(robot)