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
import time
from datetime import datetime
import cv2
import signal
import websockets
import numpy as np
import sys
import logging
import os
from device import getNextWebcamPath

# shared variables
frame_shape = (360, 640)
tx_interval = 1 / 60

camera_entity = None
frame_lock = Lock()
color_buf = None
depth_buf = None
frame2_lock = Lock()
color2_buf = None
cam2 = None
cam2_enable = Value(c_bool, False)
cam2_reserve = Value(c_bool, False)

motor_values = None
sensor_values = None # [voltage, sensor1, sensor2, ...]
sensor_length = Value(c_int, 0)

main_loop = None
_running = Value(c_bool, True)
last_rx_time = Array(c_char, 100)
comms_task = None

encoding_params = [int(cv2.IMWRITE_PNG_COMPRESSION), 1,
                   cv2.IMWRITE_PNG_STRATEGY, cv2.IMWRITE_PNG_STRATEGY_RLE]

async def sender(websocket):
  # we can use this in order to prevent data transfer latencies or data synchronization issues
  global cam2
  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, dtype=np.uint16).reshape((h, w))
  color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
  last_tx_time = None
  # right now a second camera just causes the jetson to overload
  # if cam2_enable.value and cam2_reserve.value and cam2 is None:
  #   cam_path = getNextWebcamPath()
  #   if cam_path is not None:
  #     cam2 = cv2.VideoCapture(cam_path)
  #     cam2.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
  #     cam2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
  #     cam2.set(cv2.CAP_PROP_FPS, 30)

  while _running.value:
    try:
      # throttle communication so that we don't bombard the socket connection
      curr_time = datetime.now()
      dt = tx_interval if last_tx_time is None else (curr_time - last_tx_time).total_seconds()
      if dt < tx_interval:
        await asyncio.sleep(tx_interval - dt)
        last_tx_time = datetime.now()
      else:
        last_tx_time = curr_time

      color, depth = None, None
      if camera_entity is not None:
        color, depth = camera_entity.read()
        _, color = cv2.imencode('.png', color, encoding_params)
        _, depth = cv2.imencode('.png', depth, encoding_params)
      if color is None or depth is None: # we don't have an image or controlled camera
        # frame_lock.acquire()
        _, color = cv2.imencode('.png', color_np, encoding_params)
        _, depth = cv2.imencode('.png', depth_np, encoding_params)
        # frame_lock.release()
        
      color2 = None
      if cam2_enable.value:
        if cam2_reserve.value:
          if cam2 is not None:
            retval, color2 = cam2.read()
            if retval:
              _, color2 = cv2.imencode('.png', color2, encoding_params)
        else:
          # frame2_lock.acquire()
          _, color2 = cv2.imencode('.png', color2_np, encoding_params)
          # frame2_lock.release()

      sensor_values.acquire()
      sensors = sensor_values[:sensor_length.value]
      sensor_values.release()

      frames = [pickle.dumps(color, 0), pickle.dumps(depth, 0)]
      if cam2_enable.value and color2 is not None:
        frames += [pickle.dumps(color2, 0)]
      msg = json.dumps({
        "lengths": [len(f) for f in frames],
        "timestamp": datetime.isoformat(curr_time),
        "sensors": [int(x) for x in sensors[1:]],
        "voltage": int(sensors[0])
      }).encode("utf-8")
      for f in frames:
        msg += f

      await websocket.send(msg)
    except websockets.ConnectionClosed:
      logging.warning(datetime.isoformat(datetime.now()) + " Connection closed.")
      last_tx_time = None
      # await asyncio.sleep(1)
      break

async def receiver(websocket):
  while _running.value:
    try:
      msg = await websocket.recv()
      motors = json.loads(msg.decode("utf-8"))["motors"]
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
      logging.warning(datetime.isoformat(datetime.now()) + " Connection closed.")
      motor_values.acquire()
      motor_values[:] = [0] * 10
      motor_values.release()
      # last_rx_time.acquire()
      # last_rx_time.value = "".encode()
      # last_rx_time.release()
      # await asyncio.sleep(1)
      break

async def handle_websocket(websocket, path):
  recv_task = main_loop.create_task(receiver(websocket))
  send_task = main_loop.create_task(sender(websocket))
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

def comms_worker(port, run, cam, cbuf, dbuf, flock, cam2_r, cbuf2, cam2_en, flock2, mvals, svals, ns):
  global main_loop, _running
  global camera_entity, color_buf, depth_buf, frame_lock
  global cam2_reserve, color2_buf, cam2_enable, frame2_lock
  global motor_values, sensor_values, sensor_length

  camera_entity = cam
  color_buf = cbuf
  depth_buf = dbuf
  frame_lock = flock

  cam2_reserve = cam2_r
  color2_buf = cbuf2
  cam2_enable = cam2_en
  frame2_lock = flock2

  motor_values = mvals
  sensor_values = svals
  sensor_length = ns

  _running = run
  main_loop = asyncio.new_event_loop()
  request_task = main_loop.create_task(request_handler("0.0.0.0", port))
  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(request_task)
  except (KeyboardInterrupt,):
    _running.value = False
    main_loop.stop()
  finally:
    main_loop.run_until_complete(main_loop.shutdown_asyncgens())
    main_loop.close()

def start(port=9999, robot=None, realsense=None, secondaryCam=False):
  """Start a local area network connection

  Args:
      port (int, optional): Websocket port. Defaults to 9999.
      robot (_type_, optional): Vex Serial entity. Defaults to None.
      realsense (_type_, optional): _description_. Defaults to None.
      secondaryCam (bool, optional): Reserve the /dev/video* cam port on LAN. Don't set to True if you already opened a camera. Defaults to False.
  """
  global comms_task
  global robot_entity, motor_values, sensor_values, sensor_length
  global camera_entity, color_buf, depth_buf, color2_buf
  robot_entity = robot
  camera_entity = realsense
  if secondaryCam:
    cam2_enable.value = True
    cam2_reserve.value = True

  if robot_entity is not None:
    motor_values  = robot_entity._motor_values._data
    sensor_values = robot_entity._sensor_values._data
    sensor_length = robot_entity._num_sensors
  else:
    motor_values  = Array(c_int, 10)
    sensor_values = Array(c_int, 21)
    sensor_length.value = 1

  color_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)
  depth_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 2)
  color2_buf = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)

  comms_task = Process(target=comms_worker, args=(
    port, _running,
    camera_entity, color_buf, depth_buf, frame_lock,
    cam2_reserve, color2_buf, cam2_enable, frame2_lock,
    motor_values, sensor_values, sensor_length))
  comms_task.start()

def stop():
  global comms_task
  _running.value = False
  time.sleep(0.3)
  comms_task.kill()

def set_frame(color: np.ndarray, depth: np.ndarray, cam2_color: np.ndarray=None):
  """Set the outgoing color, depth frames

  Args:
      color (np.ndarray): Realsense Camera color frame
      depth (np.ndarray): Realsense Camera depth frame
      cam2_color (np.ndarray): color frame of the second camera
  """
  if color is None or depth is None: return
  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8).reshape((h, w, 3))
  depth_np = np.frombuffer(depth_buf, np.uint16).reshape((h, w))
  # frame_lock.acquire()
  np.copyto(color_np, color)
  np.copyto(depth_np, depth)
  # frame_lock.release()
  if cam2_color is not None:
    color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
    # frame2_lock.acquire()
    cam2_enable.value = True
    np.copyto(color2_np, cam2_color)
    # frame2_lock.release()

def set_secondary_frame(color: np.ndarray):
  """Set the outgoing color frame of the second camera

  Args:
      color (np.ndarray): color frame of the second camera
  """
  h, w = frame_shape
  if color is not None:
    color2_np = np.frombuffer(color2_buf, np.uint8).reshape((h, w, 3))
    frame2_lock.acquire()
    cam2_enable.value = True
    np.copyto(color2_np, color)
    frame2_lock.release()

def write(values, voltage_level=None):
  """Send sensor values and voltage to remote location

  Args:
      values (List[int]): sensor values
      voltage_level (int, optional): voltage of the battery in mV. Defaults to None.
  """
  values = [int(x) for x in values]
  sensor_values.acquire()
  nsensors = sensor_length.value = min(20, len(values)) + 1
  sensor_values[1:nsensors] = values
  if voltage_level is not None:
    sensor_values[0] = int(voltage_level)
  else:
    sensor_values[0] = 0
  sensor_values.release()

def readtime():
  last_rx_time.acquire()
  rxtime = last_rx_time.value.decode()
  if len(rxtime) > 0:
    rxtime = datetime.strptime(rxtime, "%Y-%m-%dT%H:%M:%S.%f")
  else:
    rxtime = None
  last_rx_time.release()
  return rxtime

def check_alive():
  rxtime = readtime()
  if rxtime is None or (datetime.now() - rxtime).total_seconds() > 0.5: # timeout 0.5s before setting motors to 0
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
  values = motor_values[:]
  motor_values.release()
  return values
  
def running():
  global _running
  return _running.value