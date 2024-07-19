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
import cv2
import qoi

# shared variables
frame_shape = (360, 640)
tx_interval = 1 / 120

class RemoteByteBuf:
  def __init__(self, size=None):
    self.lock = Lock()
    self.size = size
    self.buf = None if (size is None) else RawArray(c_uint8, int(np.prod(self.size)))
    self.len = Value(c_int, 0)
    self.en = Value(c_bool, False)

    self.encode = lambda frame: frame
    self.decode = lambda frame: frame

  def sync_process(self, lock, buf, len, en):
    self.lock = lock
    self.buf = buf
    self.len = len
    self.en = en

  def numpy(self, dtype=np.uint8):
    return np.frombuffer(self.buf, dtype=dtype)
  
  def enabled(self):
    return self.en.value
  
  def copyfrom(self, data):
    data = data.flatten() if isinstance(data, np.ndarray) else np.frombuffer(data, np.uint8)
    self.lock.acquire()
    if len(data) == np.prod(self.size):
      np.copyto(self.numpy(), data)
    else:
      np.copyto(self.numpy()[:len(data)], data)
    self.len.value = len(data)
    self.en.value = True
    self.lock.release()

  def asbytes(self):
    self.lock.acquire()
    if self.en.value == True:
      bytestr = self.numpy()[:self.len.value].tobytes()
      self.en.value = False
    else:
      bytestr = None
    self.lock.release()
    return bytestr

_color_encoding_parameters = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

motor_values = None
sensor_values = None # [voltage, sensor1, sensor2, ...]
sensor_length = Value(c_int, 0)

main_loop = None
_running = Value(c_bool, True)
last_rx_time = Array(c_char, 100)
_stream_host = Array(c_int, 4)

rxtx_task = None
send_task1 = None
send_task2 = None
send_task3 = None

color_buf = None
depth_buf = None
color_buf2 = None

# because the jetson doesn't properly support wchar
def ip2l(x): return [int(b) for b in x.split('.')]
def l2ip(x): return "%d.%d.%d.%d" % tuple(x)

async def streamer(port, msg_buf):
  while _running.value:
    if not msg_buf.enabled():
      await asyncio.sleep(0.01)
      continue

    _stream_host.acquire()
    host = l2ip(_stream_host) # always update to the latest ipv4 host
    _stream_host.release()
    try:
      if host != "0.0.0.0":
        msg = msg_buf.asbytes()
        async with websockets.connect("ws://" + host + ":" + str(port)) as websocket:
          await websocket.send(msg)
    except Exception as e:
      await asyncio.sleep(0.01)

def streamer_worker(host, port, run, lock, buf, len, en):
  global main_loop, _running, _stream_host
  msg_buf = RemoteByteBuf()
  msg_buf.sync_process(lock, buf, len, en)
  _stream_host = host

  _running = run
  main_loop = asyncio.new_event_loop()
  stream_task = main_loop.create_task(streamer(port, msg_buf))

  for signo in [signal.SIGINT, signal.SIGTERM]:
    main_loop.add_signal_handler(signo, stream_task.cancel)
  
  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(stream_task)
  except (KeyboardInterrupt,):
    _running.value = False
    main_loop.stop()
  finally:
    main_loop.run_until_complete(main_loop.shutdown_asyncgens())
    main_loop.close()

async def sender(websocket):
  last_tx_time = None

  while _running.value:
    try:
      curr_time = time.time()
      dt = tx_interval if last_tx_time is None else (curr_time - last_tx_time)
      if dt < tx_interval:
        await asyncio.sleep(tx_interval - dt) # throttle to prevent overload
        last_tx_time = time.time()
      else:
        last_tx_time = curr_time

      sensor_values.acquire()
      sensors = sensor_values[:sensor_length.value]
      sensor_values.release()

      msg = json.dumps({
        "timestamp": datetime.isoformat(datetime.now()),
        "sensors": [int(x) for x in sensors[1:]],
        "voltage": int(sensors[0])
      })
      await websocket.send(msg)
    except websockets.ConnectionClosed:
      # logging.warning(datetime.isoformat(datetime.now()) + " Connection closed.")
      last_tx_time = None
      await asyncio.sleep(1) # just wait forever...

async def receiver(websocket):
  while _running.value:
    try:
      msg = await websocket.recv()
      msg = json.loads(msg.decode("utf-8"))
      _stream_host.acquire()
      _stream_host[:] = ip2l(msg["ipv4"])
      _stream_host.release()

      motors = msg["motors"]
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
      # logging.warning(datetime.isoformat(datetime.now()) + " Connection closed.")
      motor_values.acquire()
      motor_values[:] = [0] * 10
      motor_values.release()
      await asyncio.sleep(1)

async def handle_rxtx(websocket, path):
  recv_task = main_loop.create_task(receiver(websocket))
  send_task = main_loop.create_task(sender(websocket))
  await asyncio.gather(recv_task, send_task)

async def request_handler(host, port):
  async with websockets.serve(handle_rxtx, host, port):
    try:
      await asyncio.Future()
    except asyncio.exceptions.CancelledError:
      logging.info("Closing gracefully.")
      return
    except Exception as e:
      logging.error(e)
      sys.exit(1)

def rxtx_worker(host, port, run, mvals, svals, slen):
  global main_loop, _running, _stream_host
  global motor_values, sensor_values, sensor_length
  motor_values = mvals
  sensor_values = svals
  sensor_length = slen
  _stream_host = host

  _running = run
  main_loop = asyncio.new_event_loop()
  rxtx_task = main_loop.create_task(request_handler("0.0.0.0", port))
  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(rxtx_task)
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
  global robot_entity, motor_values, sensor_values, sensor_length
  global color_buf, depth_buf, color_buf2
  global rxtx_task, send_task1, send_task2, send_task3
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
  qoi_size = lambda h, w, c: 14 + h * w * (c + 1) + 8 + 2
  cbuf = RemoteByteBuf(np.prod((frame_shape[0], frame_shape[1], 3)))
  dbuf = RemoteByteBuf(qoi_size(frame_shape[0], frame_shape[1] // 2, 4))
  cbuf2 = RemoteByteBuf(np.prod((frame_shape[0], frame_shape[1], 3)))

  rxtx_task = Process(target=rxtx_task, args=(
    _stream_host, port, _running, motor_values, sensor_values, sensor_length))
  rxtx_task.start()

  send_task1 = Process(target=streamer_worker, args=(
    _stream_host, port-1, _running, cbuf.lock, cbuf.buf, cbuf.len, cbuf.en))
  send_task2 = Process(target=streamer_worker, args=(
    _stream_host, port-2, _running, dbuf.lock, dbuf.buf, dbuf.len, dbuf.en))
  send_task3 = Process(target=streamer_worker, args=(
    _stream_host, port-3, _running, cbuf2.lock, cbuf2.buf, cbuf2.len, cbuf2.en))

  send_task1.start()
  send_task2.start()
  send_task3.start()

  color_buf = cbuf
  depth_buf = dbuf
  color_buf2 = cbuf2

def stop():
  global send_task1, send_task2, send_task3, rxtx_task
  _running.value = False
  time.sleep(0.3)
  send_task1.kill()
  send_task2.kill()
  send_task3.kill()
  rxtx_task.kill()

def set_frames(color: np.ndarray=None, depth: np.ndarray=None, color2: np.ndarray=None):
  """Set the outgoing color, depth frames

  Args:
      color (np.ndarray, optional): Realsense Camera color frame. Defaults to None.
      depth (np.ndarray, optional): Realsense Camera depth frame. Defaults to None.
      color2 (np.ndarray, optional): second camera color frame. Defaults to None.
  """
  h, w = frame_shape
  if color is not None and depth is not None:
    color = pickle.dumps(cv2.imencode('.jpg', color, _color_encoding_parameters), 0)
    depth = qoi.encode(depth.view(np.uint8).reshape((h, w // 2, 4)))
  if color2 is not None:
    color2 = pickle.dumps(cv2.imencode('.jpg', color2, _color_encoding_parameters), 0)
  if color is not None and depth is not None:
    color_buf.copyfrom(color)
    depth_buf.copyfrom(depth)
  if color2 is not None:
    color_buf2.copyfrom(color2)

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