
import asyncio
from ctypes import ( c_double, c_int, c_bool, c_uint8, c_uint16, c_wchar_p )
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
import cv2
import websockets
import numpy as np
import signal
import sys
# import uvloop
# asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

from .device import RealsenseCamera

# shared variables
_motor_values = None
_sensor_values = None
_num_sensors = None
_last_rx_time = None
_rxtx_start_time = time.time()

_frame_shape = (360, 640)
_frame_color = None
_frame_depth = None
_frame_lock = Lock()
_frame_color_np = None
_frame_depth_np = None
_color_encoding_parameters = [int(cv2.IMWRITE_PNG_COMPRESSION), 1]
_depth_encoding_parameters = [int(cv2.IMWRITE_PNG_COMPRESSION), 1]

main_loop = None
_running = Value(c_bool, False)
_rxtx_task = None
_rgbd_task = None
_robot_entity = None
_camera_entity = None
_rgbd_ms_interval = 0.02
_host = None
_port = None
_stream_host = None
_stream_port = None

_rxtx_process = None
_rgbd_process = None

async def streamer_source():
  # we can use this in order to prevent data transfer latencies or data synchronization issues
  global _running, _camera_entity, _stream_port, main_loop
  _stream_host.acquire()
  host = _stream_host.value
  port = _stream_port.value
  _stream_host.release()

  h, w = _frame_shape
  _frame_color_np = np.frombuffer(_frame_color, np.uint8).reshape((h, w, 3))
  _frame_depth_np = np.frombuffer(_frame_depth, np.uint16).reshape((h, w))

  while _running.value:
    color, depth = None, None
    if _camera_entity is not None:
      color, depth = _camera_entity.read()
    if color is None or depth is None: # we don't have an image or controlled camera
      time.sleep(_rgbd_ms_interval) # throttle in case we don't account for camera being opened
      _frame_lock.acquire()
      color = np.copy(_frame_color_np)
      depth = np.copy(_frame_depth_np)
      _frame_lock.release()
      h, w = _frame_shape

    # consider testing the following using async with main_loop.create_task
    _, color = cv2.imencode('.png', color, _color_encoding_parameters)
    _, depth = cv2.imencode('.png', depth, _depth_encoding_parameters)
    data = pickle.dumps((color, depth), 0)
    try:
      async with websockets.connect(f"ws://{host}:{port}") as websocket:
        await websocket.send(data)
    except Exception as e:
      # try to get the latest ipv4
      _stream_host.acquire()
      host = _stream_host.value
      port = _stream_port.value
      _stream_host.release()

async def handle_rxtx(req, websocket):
  global _last_rx_time, _sensor_values, _num_sensors
  global _motor_values
  global _rxtx_start_time

  curr_time = time.time()
  motor_values = req["motors"]
  if len(motor_values) == 10:
    _motor_values.acquire()
    _motor_values[:] = motor_values
    _motor_values.release()
    _last_rx_time.acquire()
    _last_rx_time.value = curr_time
    _last_rx_time.release()

  # send back sensor values over socket
  sensor_values = []
  _sensor_values.acquire()
  nsensors = _num_sensors.value
  if nsensors > 0:
    sensor_values = _sensor_values[:nsensors]
  _sensor_values.release()
  timestamp = curr_time - _rxtx_start_time
  # _rxtx_start_time = time.time()
  await websocket.send(json.dumps({ "sensors": sensor_values, "timestamp": timestamp }))

async def handle_request(websocket, path):
  global main_loop, _stream_host, _stream_port
  async for req in websocket:
    request = json.loads(req)
    if "motors" in request:
      await handle_rxtx(request, websocket)
    if "ipv4" in req:
      _stream_host.acquire()
      _stream_host.value = request["ipv4"]
      _stream_host.release()

async def request_handler(host, port):
  async with websockets.serve(handle_request, host, port):
    try:
      await asyncio.Future()
    except asyncio.exceptions.CancelledError:
      print("Closing gracefully.")
      return
    except Exception as e:
      print(e)
      sys.exit(1)

def run_async_rxtx(host, port, shost, sport, run, mvals, svals, ns):
  global _rxtx_task, _running, main_loop, _stream_host, _stream_port
  global _host, _port, _motor_values, _sensor_values, _num_sensors
  
  _host = host
  _port = port
  _stream_host = shost
  _stream_port = sport
  _running = run
  _motor_values = mvals
  _sensor_values = svals
  _num_sensors = ns

  if main_loop is None:
    main_loop = asyncio.new_event_loop()
  _rxtx_task = main_loop.create_task(request_handler(host, port))

  for signo in [signal.SIGINT, signal.SIGTERM]:
    main_loop.add_signal_handler(signo, _rxtx_task.cancel)

  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(_rxtx_task)
  except (KeyboardInterrupt,):
    _running.value = False
    main_loop.stop()
  finally:
    main_loop.run_until_complete(main_loop.shutdown_asyncgens())
    main_loop.close()

def run_async_rgbd(host, port, run, fshape, fcolor, fdepth, flock, cam):
  global _rgbd_task, _running, main_loop, _camera_entity
  global _stream_host, _stream_port
  global _frame_shape, _frame_color, _frame_depth, _frame_lock

  _stream_host = host
  _stream_port = port
  _running = run
  _frame_shape = fshape
  _frame_color = fcolor
  _frame_depth = fdepth
  _frame_lock = flock
  _camera_entity = cam

  if main_loop is None:
    main_loop = asyncio.new_event_loop()
  _rgbd_task = main_loop.create_task(streamer_source())

  for signo in [signal.SIGINT, signal.SIGTERM]:
    main_loop.add_signal_handler(signo, _rgbd_task.cancel)

  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(_rgbd_task)
  except (KeyboardInterrupt,):
    _running.value = False
    main_loop.stop()
  finally:
    main_loop.run_until_complete(main_loop.shutdown_asyncgens())
    main_loop.close()

def start(port=9999, frame_shape=(360, 640), target=None, camera=None):
  global _robot_entity, _camera_entity, _running, _stream_host, _stream_port
  global _motor_values, _sensor_values, _num_sensors, _last_rx_time
  global _host, _port, _frame_shape, _frame_color, _frame_depth, _frame_lock
  global _rxtx_process, _rgbd_process
  _robot_entity = target
  _camera_entity = camera

  if _robot_entity is not None:
    _motor_values  = _robot_entity._motor_values._data
    _sensor_values = _robot_entity._sensor_values._data
    _num_sensors   = _robot_entity._num_sensors
  else:
    _motor_values  = Array(c_int, 10)
    _sensor_values = Array(c_int, 20)
    _num_sensors   = Value(c_int, 0)
  _last_rx_time = Value(c_double, time.time())

  _host = "0.0.0.0"
  _port = port
  _running = Value(c_bool, True)
  _stream_host = Value(c_wchar_p, _host)
  _stream_port = Value(c_int, port + 1)
  _frame_shape = frame_shape
  _frame_color = RawArray(c_uint8, frame_shape[0] * frame_shape[1] * 3)
  _frame_depth = RawArray(c_uint16, frame_shape[0] * frame_shape[1])
  _frame_lock = Lock()

  _rgbd_process = Process(target=run_async_rgbd, args=(
    _stream_host, _stream_port, _running,
    _frame_shape, _frame_color, _frame_depth, _frame_lock, _camera_entity))
  _rgbd_process.start()

  _rxtx_process = Process(target=run_async_rxtx, args=(
    _host, _port, _stream_host, _stream_port, _running,
    _motor_values, _sensor_values, _num_sensors))
  _rxtx_process.start()

def stop():
  global _rxtx_process, _rgbd_process, _running
  _running.value = False
  time.sleep(0.3)
  _rxtx_process.terminate()
  _rgbd_process.terminate()
  _rxtx_process.join()
  _rgbd_process.join()

def sig_handler(signum, frame):
  if signum == signal.SIGINT:
    stop()
    sys.exit()

signal.signal(signal.SIGINT, sig_handler)

def set_frame(color: np.ndarray, depth: np.ndarray):
  global _frame_lock, _frame_color, _frame_depth
  global _frame_color_np, _frame_depth_np
  if color is None or depth is None: return
  if _frame_color_np is None and _frame_color is not None:
    h, w = _frame_shape
    _frame_color_np = np.frombuffer(_frame_color, np.uint8).reshape((h, w, 3))
    _frame_depth_np = np.frombuffer(_frame_depth, np.uint16).reshape((h, w))
  _frame_lock.acquire()
  np.copyto(_frame_color_np, color)
  np.copyto(_frame_depth_np, depth)
  _frame_lock.release()

def write(sensor_values, voltage_level=None):
  """Send motor values to remote location

  Args:
      sensor_values (List[int]): sensor values
      voltage_level (int, optional): Voltage of the robot. Defaults to None.
  """
  global _sensor_values
  sensor_values = [int(x) for x in sensor_values]
  _sensor_values.acquire()
  nsensors = _num_sensors.value = min(20, len(sensor_values))
  _sensor_values[:nsensors] = sensor_values
  _sensor_values.release()

def readtime():
  _last_rx_time.acquire()
  rxtime = _last_rx_time.value
  _last_rx_time.release()
  return rxtime

def check_alive():
  rxtime = readtime()
  if (time.time() - rxtime) > 0.5: # timeout 0.5s before setting motors to 0
    _motor_values[:] = [0] * len(_motor_values) # just in case an error occured
    _motor_values.acquire()
    _motor_values[:] = [0] * len(_motor_values)
    _motor_values.release()
    return False
  else:
    return True

def read():
  """Return motor values that are read in

  Returns:
      List[int]: motor values
  """
  check_alive()
  _motor_values.acquire()
  motor_values = _motor_values[:]
  _motor_values.release()
  return motor_values
  
def running():
  global _running
  return _running.value

if __name__ == "__main__":
  start(camera=RealsenseCamera(autostart=False))
  sensor_values = [0] * 2
  while running():
    check_alive()
    time.sleep(0.02)
    sensor_values[0] += 1
    sensor_values[0] %= 127
    write(sensor_values)
    # print(read())
  stop()