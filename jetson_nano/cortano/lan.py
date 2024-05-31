import asyncio
from ctypes import ( c_double, c_int, c_bool, c_uint8, c_uint16, c_char )
import json
from multiprocessing import (
  Process,
  Lock,
  Array,
  RawArray,
  Value
)
import pickle
import qoi
import time
from datetime import datetime
import cv2
import signal
import websockets
import numpy as np
import sys
import logging

from .device import RealsenseCamera

# shared variables
frame_shape = (360, 640)
tx_interval = 1 / 60

camera_entity = None
camera2_entity = None
frame_lock = Lock()
color_buf = None
depth_buf = None
frame2_lock = Lock()
color2_buf = None
cam2_enable = Value(c_bool, False)

motor_values = None
sensor_values = None # [voltage, sensor1, sensor2, ...]
sensor_length = Value(c_int, 0)

main_loop = None
running = Value(c_bool, False)
last_rx_time = Array(c_char, 100)
comms_task = None

# because the jetson doesn't properly support wchar
def ip2l(x): return [int(b) for b in x.split('.')]
def l2ip(x): return "%d.%d.%d.%d" % tuple(x)

async def sender(websocket):
  # we can use this in order to prevent data transfer latencies or data synchronization issues
  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
  last_tx_time = None

  while running.value:
    try:
      # throttle communication so that we don't bombard the socket connection
      curr_time = datetime.now()
      dt = tx_interval if last_tx_time is None else (curr_time - last_tx_time).total_seconds()
      if dt < tx_interval:
        time.sleep(dt - tx_interval)
        last_tx_time = datetime.now()
      else:
        last_tx_time = curr_time

      color, depth = None, None
      if camera_entity is not None:
        color, depth = camera_entity.read()
        color = qoi.encode(color)
        depth = qoi.encode(depth)
      if color is None or depth is None: # we don't have an image or controlled camera
        frame_lock.acquire()
        color = qoi.encode(color_np)
        depth = qoi.encode(depth_np)
        frame_lock.release()
        
      color2 = None
      if cam2_enable.value:
        if camera2_entity is not None:
          _, color2 = camera2_entity.read()
          color2 = qoi.encode(color2)
        if color2 is None:
          frame2_lock.acquire()
          color2 = qoi.encode(color2_np)
          frame2_lock.release()

      sensor_values.acquire()
      sensors = sensor_values[:sensor_length.value]
      sensor_values.release()

      # consider testing the following using async with main_loop.create_task
      frames = [pickle.dumps(color, 0), pickle.dumps(depth, 0)]
      if cam2_enable.value:
        frames += [pickle.dumps(color2, 0)]
      data = json.dumps({
        "lengths": [len(f) for f in frames],
        "timestamp": datetime.isoformat(curr_time),
        "sensors": sensors[1:],
        "voltage": sensors[0]
      }).decode("utf-8")
      for f in frames:
        data += f

      await websocket.send(data)
    except websockets.ConnectionClosed:
      logging.warning(datetime.isoformat(datetime.now()), "Connection closed, attempting to reestablish...")
      last_tx_time = None
      await asyncio.sleep(1)

async def receiver(websocket):
  global motor_values, last_rx_time
  while running.value:
    try:
      msg = await websocket.recv()
      motors = json.loads(msg)["motors"]
      if len(motors) == 10:
        motor_values.acquire()
        motor_values[:] = motors
        motor_values.release()
        last_rx_time.acquire()
        last_rx_time.value = datetime.isoformat(datetime.now()).encode()
        last_rx_time.release()
    except ValueError:
      logging.error("Invalid data received:", msg)
    except websockets.ConnectionClosed:
      logging.warning(datetime.isoformat(datetime.now()), "Connection closed, attempting to reestablish...")
      motor_values.acquire()
      motor_values[:] = [0] * 10
      motor_values.release()
      last_rx_time.acquire()
      last_rx_time.value = "".encode()
      last_rx_time.release()
      await asyncio.sleep(1)

async def handle_websocket(websocket, path):
  recv_task = asyncio.create_task(receiver(websocket))
  send_task = asyncio.create_task(sender(websocket))
  await asyncio.gather(recv_task, send_task)

async def request_handler(host, port):
  async with websockets.serve(handle_websocket, host, port):
    try:
      await asyncio.Future()
    except asyncio.exceptions.CancelledError:
      logging.info("Closing gracefully.")
      return
    except Exception as e:
      logging.error(e)
      sys.exit(1)

def comms_worker(port, run, cam, cbuf, dbuf, flock, cam2, cbuf2, cam2_en, flock2, mvals, svals, ns):
  global main_loop, running
  global camera_entity, color_buf, depth_buf, frame_lock
  global camera2_entity, color2_buf, cam2_enable, frame2_lock
  global motor_values, sensor_values, sensor_length

  camera_entity = cam
  color_buf = cbuf
  depth_buf = dbuf
  frame_lock = flock

  camera2_entity = cam2
  color2_buf = cbuf2
  cam2_enable = cam2_en
  frame2_lock = flock2

  motor_values = mvals
  sensor_values = svals
  sensor_length = ns

  running = run
  main_loop = asyncio.new_event_loop()
  request_task = main_loop.create_task(request_handler("0.0.0.0", port))
  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(request_task)
  except (KeyboardInterrupt,):
    running.value = False
    main_loop.stop()
  finally:
    main_loop.run_until_complete(main_loop.shutdown_asyncgens())
    main_loop.close()

def start(port=9999, robot=None, realsense: RealsenseCamera=None, camera: cv2.VideoCapture=None):
  global running, comms_task
  global robot_entity, motor_values, sensor_values
  global camera_entity, camera2_entity, color_buf, depth_buf, color2_buf
  robot_entity = robot
  camera_entity = realsense
  camera2_entity = camera
  if camera2_entity is not None:
    cam2_enable.value = True

  if robot_entity is not None:
    motor_values  = robot_entity._motor_values._data
    sensor_values = robot_entity._sensor_values._data
    sensor_length.value = robot_entity._num_sensors
  else:
    motor_values  = Array(c_int, 10)
    sensor_values = Array(c_int, 21)

  color_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)
  depth_buf = RawArray(c_uint16, frame_shape[0] * frame_shape[1])
  color2_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)
  frame_lock = Lock()

  comms_task = Process(target=comms_worker, args=(
    port, running,
    camera_entity, color_buf, depth_buf, frame_lock,
    camera2_entity, color2_buf, cam2_enable, frame2_lock,
    motor_values, sensor_values, sensor_length))
  comms_task.start()

def stop():
  global comms_task, running
  running.value = False
  time.sleep(0.3)
  comms_task.kill() # just brute force kill

def sig_handler(signum, frame):
  if signum == signal.SIGINT:
    stop()
    sys.exit()

signal.signal(signal.SIGINT, sig_handler)

def set_frame(color: np.ndarray, depth: np.ndarray, cam2_color: np.ndarray=None):
  if color is None or depth is None: return
  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  frame_lock.acquire()
  np.copyto(color_np, color)
  np.copyto(depth_np, depth)
  frame_lock.release()
  if cam2_color is not None:
    color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
    frame2_lock.acquire()
    cam2_enable.value = True
    np.copyto(color2_np, cam2_color)
    frame2_lock.release()

def set_secondary_frame(color: np.ndarray):
  h, w = frame_shape
  if color is not None:
    color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
    frame2_lock.acquire()
    cam2_enable.value = True
    np.copyto(color2_np, color)
    frame2_lock.release()

def write(values, voltage_level=None):
  """Send motor values to remote location

  Args:
      values (List[int]): sensor values
      voltage_level (int, optional): Voltage of the robot. Defaults to None.
  """
  values = [int(x) for x in values]
  sensor_values.acquire()
  nsensors = sensor_length.value = min(20, len(sensor_values))
  sensor_values[:nsensors] = values
  sensor_values.release()

def readtime():
  last_rx_time.acquire()
  rxtime = last_rx_time.value.decode()
  last_rx_time.release()
  return rxtime

def check_alive():
  rxtime = readtime()
  if (datetime.now() - rxtime).total_seconds() > 0.5: # timeout 0.5s before setting motors to 0
    motor_values[:] = [0] * len(motor_values) # just in case an error occured
    motor_values.acquire()
    motor_values[:] = [0] * len(motor_values)
    motor_values.release()
    return False
  else:
    return True

def read():
  """Return motor values that are read in

  Returns:
      List[int]: motor values
  """
  check_alive()
  motor_values.acquire()
  motor_values = motor_values[:]
  motor_values.release()
  return motor_values
  
def running():
  global running
  return running.value
