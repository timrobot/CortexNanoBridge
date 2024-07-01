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
import signal
import websockets
import numpy as np
import sys
import logging
import os
import qoi

# shared variables
frame_shape = (360, 640)
tx_interval = 1 / 60

frame_lock = Lock()
color_buf = None
depth_buf = None
color2_buf = None
color_len = Value(c_int, 0)
depth_len = Value(c_int, 0)
color2_len = Value(c_int, 0)
color2_en = Value(c_bool, False)

motor_values = None
sensor_values = None # [voltage, sensor1, sensor2, ...]
sensor_length = Value(c_int, 0)

main_loop = None
_running = Value(c_bool, True)
last_rx_time = Array(c_char, 100)
comms_task = None
send_task1 = None
send_task2 = None
recv_task = None

async def sender(websocket):
  # we can use this in order to prevent data transfer latencies or data synchronization issues
  color_np = np.frombuffer(color_buf, np.uint8)
  depth_np = np.frombuffer(depth_buf, np.uint8)
  color2_np = np.frombuffer(color2_buf, np.uint8)
  last_tx_time = None

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

      frame_str = None
      frame_len = []
      frame_lock.acquire()
      frame_len.append(color_len.value)
      frame_len.append(depth_len.value)
      frame_str = color_np[:color_len.value].tobytes() + depth_np[:depth_len.value].tobytes()
      if color2_en.value:
        frame_len.append(color2_len.value)
        frame_str += color2_np[:color2_len.value].tobytes()
      frame_lock.release()

      sensor_values.acquire()
      sensors = sensor_values[:sensor_length.value]
      sensor_values.release()

      msg = json.dumps({
        "lengths": frame_len,
        "timestamp": datetime.isoformat(curr_time),
        "sensors": [int(x) for x in sensors[1:]],
        "voltage": int(sensors[0])
      }).encode("utf-8")
      msg += frame_str
      await websocket.send(msg)
    except websockets.ConnectionClosed:
      logging.warning(datetime.isoformat(datetime.now()) + " Connection closed.")
      last_tx_time = None
      await asyncio.sleep(1)

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
      await asyncio.sleep(1)

async def handle_send(websocket, path):
  global send_task
  send_task = main_loop.create_task(sender(websocket))
  await asyncio.gather(send_task)

async def handle_recv(websocket, path):
  global recv_task
  recv_task = main_loop.create_task(receiver(websocket))
  await asyncio.gather(recv_task)

async def request_handler(host, port, coro):
  async with websockets.serve(coro, host, port):
    try:
      await asyncio.Future()
    except asyncio.exceptions.CancelledError:
      logging.info("Closing gracefully.")
      return
    except Exception as e:
      logging.error(e)
      sys.exit(1)

def recv_worker(port, run, mvals):
  global main_loop, _running
  global motor_values
  motor_values = mvals

  _running = run
  main_loop = asyncio.new_event_loop()
  recv_task = main_loop.create_task(request_handler("0.0.0.0", port, handle_recv))
  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(recv_task)
  except (KeyboardInterrupt,):
    _running.value = False
    main_loop.stop()
  finally:
    main_loop.run_until_complete(main_loop.shutdown_asyncgens())
    main_loop.close()

def send_worker(port, run, flock, cbuf, clen, dbuf, dlen, cbuf2, clen2, c2en, svals, ns):
  global main_loop, _running
  global color_buf, color_len, depth_buf, depth_len, frame_lock
  global color2_buf, color2_len, color2_en
  global sensor_values, sensor_length

  frame_lock = flock
  color_buf = cbuf
  color_len = clen
  depth_buf = dbuf
  depth_len = dlen
  color2_buf = cbuf2
  color2_len = clen2
  color2_en = c2en

  sensor_values = svals
  sensor_length = ns

  _running = run
  main_loop = asyncio.new_event_loop()
  send_task = main_loop.create_task(request_handler("0.0.0.0", port, handle_send))
  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(send_task)
  except (KeyboardInterrupt,):
    _running.value = False
    main_loop.stop()
  finally:
    main_loop.run_until_complete(main_loop.shutdown_asyncgens())
    main_loop.close()

def start(port=9999, robot=None):
  """Start a local area network connection

  Args:
      port (int, optional): Websocket port. Defaults to 9999.
      robot (_type_, optional): Vex Serial entity. Defaults to None.
      realsense (_type_, optional): _description_. Defaults to None.
      secondaryCam (bool, optional): Reserve the /dev/video* cam port on LAN. Don't set to True if you already opened a camera. Defaults to False.
  """
  global comms_task
  global robot_entity, motor_values, sensor_values, sensor_length
  global color_buf, depth_buf, color2_buf
  global recv_task, send_task
  robot_entity = robot

  if robot_entity is not None:
    motor_values  = robot_entity._motor_values._data
    sensor_values = robot_entity._sensor_values._data
    sensor_length = robot_entity._num_sensors
  else:
    motor_values  = Array(c_int, 10)
    sensor_values = Array(c_int, 21)
    sensor_length.value = 1

  # we are sending over encoded strings + termination char
  max_size = lambda h, w, c: 14 + h * w * (c + 1) + 8 + 2
  color_buf = RawArray(c_uint8, max_size(frame_shape[0], frame_shape[1], 3))
  depth_buf = RawArray(c_uint8, max_size(frame_shape[0], frame_shape[1] // 2, 4))
  color2_buf = RawArray(c_uint8, max_size(frame_shape[0], frame_shape[1], 3))

  recv_task = Process(target=recv_worker, args=(
    port, _running, motor_values))
  recv_task.start()

  send_task1 = Process(target=send_worker, args=(
    port-1, _running,
    frame_lock, color_buf, color_len, depth_buf, depth_len,
    color2_buf, color2_len, color2_en,
    sensor_values, sensor_length))
  send_task1.start()

  send_task2 = Process(target=send_worker, args=(
    port-2, _running,
    frame_lock, color_buf, color_len, depth_buf, depth_len,
    color2_buf, color2_len, color2_en,
    sensor_values, sensor_length))
  send_task2.start()

def stop():
  global send_task1, send_task2, recv_task
  _running.value = False
  time.sleep(0.3)
  send_task1.kill()
  send_task2.kill()
  recv_task.kill()

def set_frames(color: np.ndarray=None, depth: np.ndarray=None, color2: np.ndarray=None):
  """Set the outgoing color, depth frames

  Args:
      color (np.ndarray, optional): Realsense Camera color frame. Defaults to None.
      depth (np.ndarray, optional): Realsense Camera depth frame. Defaults to None.
      color2 (np.ndarray, optional): second camera color frame. Defaults to None.
  """
  h, w = frame_shape
  color_np = np.frombuffer(color_buf, np.uint8)
  depth_np = np.frombuffer(depth_buf, np.uint8)
  color2_np = np.frombuffer(color2_buf, np.uint8)
  frame_lock.acquire()
  if color is not None and depth is not None:
    np.copyto(color_np, np.frombuffer(qoi.encode(color), np.uint8))
    depth = depth.view(np.uint8).reshape((h, w // 2, 4))
    np.copyto(depth_np, np.frombuffer(qoi.encode(depth), np.uint8))
  if color2 is not None:
    color2_en.value = True
    np.copyto(color2_np, color)
  frame_lock.release()

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