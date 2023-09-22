
import asyncio
from ctypes import ( c_double, c_int, c_bool )
import json
from multiprocessing import (
  Process,
  Lock,
  Manager,
  Array,
  RawArray,
  RawValue,
  Value
)
import pickle
import time
import cv2
import websockets
import numpy as np
import signal
import sys
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
_color_encoding_parameters = [int(cv2.IMWRITE_PNG_COMPRESSION), 1]
_depth_encoding_parameters = [int(cv2.IMWRITE_PNG_COMPRESSION), 1]

main_loop = None
_running = RawValue(c_bool, False)
_stream_task = None
_req_task = None
controlled_target = None
controlled_camera = None
FPS = 30
_host = None
_port = None
_prev_transmit_task = None

comm_process = None

async def send_frame(websocket, color, depth):
  global _prev_transmit_task
  _, color = cv2.imencode('.jpg', color, _color_encoding_parameters)
  _, depth = cv2.imencode('.png', depth, _depth_encoding_parameters)
  data = pickle.dumps((color, depth), 0)

  # if _prev_transmit_task is not None:
  #   await _prev_transmit_task
  #   _prev_transmit_task = None
  # _prev_transmit_task = asyncio.create_task(websocket.send(data))
  await websocket.send(data) # temp for testing

async def streamer_source(ipv4, port):
  # we can use this in order to prevent data transfer latencies or data synchronization issues
  global _running, controlled_camera
  # if controlled_camera is not None: # camera opens automagically on read()
  #   controlled_camera.open()

  # prev_request = None
  while _running.value:
    color, depth = None, None
    if controlled_camera is not None:
      color, depth = controlled_camera.read()
    if color is None or depth is None: # we don't have an image or controlled camera
      time.sleep(1 / FPS) # throttle in case we don't account for camera being opened
      _frame_lock.acquire()
      if _frame_color is not None and _frame_depth is not None:
        color, depth = _frame_color[:], _frame_depth[:]
      _frame_lock.release()
      h, w = _frame_shape
      color = np.asarray(color, np.uint8, copy=True).reshape((h, w, 3))
      depth = np.asarray(depth, np.uint16, copy=True).reshape((h, w))
    # encode takes a while, so placed on its own task to get ready for the next read (increase FPS)
    # if prev_request is not None:
    #   await prev_request
    #   prev_request = None
    async with websockets.connect(f"ws://{ipv4}:{port}") as websocket:
      await send_frame(websocket, color, depth) # temp for testing
      # prev_request = asyncio.create_task(send_frame(websocket, color, depth))

async def handle_call(req, websocket):
  global main_loop, _stream_task
  if "ipv4" in req and "port" in req:
    # create the streamer here
    ipv4 = req["ipv4"]
    port = req["port"]
    print(f"connecting to ws server on {ipv4}:{port}")
    _stream_task = main_loop.create_task(streamer_source(ipv4, port))
    # await websocket.send(json.dumps({"status": "OK"}))

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
  sensor_values = [(curr_time - _rxtx_start_time)] + sensor_values
  # _rxtx_start_time = time.time()
  await websocket.send(json.dumps(sensor_values))

async def handle_request(websocket, path):
  async for req in websocket:
    request = json.loads(req)
    if "motors" in request:
      await handle_rxtx(request, websocket)
    else:
      await handle_call(request, websocket)

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

def run_async_loop(host, port, fshape, fcolor, fdepth, flock,
                   mvals, svals, ns, cam, run):
  global _req_task, _running
  global _host, _port, _frame_shape, _frame_color, _frame_depth, _frame_lock
  global _motor_values, _sensor_values, _num_sensors, controlled_camera
  
  _host = host
  _port = port
  _frame_shape = fshape
  _frame_color = fcolor
  _frame_depth = fdepth
  _frame_lock = flock
  _motor_values = mvals
  _sensor_values = svals
  _num_sensors = ns
  controlled_camera = cam
  _running = run

  if main_loop is None:
    main_loop = asyncio.new_event_loop()
    _running.value = True
  _req_task = main_loop.create_task(request_handler(host, port))

  for signo in [signal.SIGINT, signal.SIGTERM]:
    main_loop.add_signal_handler(signo, _req_task.cancel)

  try:
    asyncio.set_event_loop(main_loop)
    main_loop.run_until_complete(_req_task)
  except (KeyboardInterrupt,):
    _running.value = False
    main_loop.stop()
  finally:
    main_loop.run_until_complete(main_loop.shutdown_asyncgens())
    main_loop.close()

def start(port=9999, frame_shape=(360, 640), target=None, camera=None):
  global controlled_target, controlled_camera, _running
  global _motor_values, _sensor_values, _num_sensors, _last_rx_time
  global _host, _port, _frame_shape, _frame_color, _frame_depth, _frame_lock
  global comm_process
  controlled_target = target
  controlled_camera = camera

  if controlled_target is not None:
    _motor_values  = controlled_target._motor_values._data
    _sensor_values = controlled_target._sensor_values._data
    _num_sensors   = controlled_target._num_sensors
  else:
    _motor_values  = Array(c_int, 10)
    _sensor_values = Array(c_int, 20)
    _num_sensors   = Value(c_int, 0)
  _last_rx_time = Value(c_double, time.time())

  _host = "0.0.0.0"
  _port = port
  _frame_shape = frame_shape
  _frame_color = RawArray(np.uint8, frame_shape[0] * frame_shape[1] * 3)
  _frame_depth = RawArray(np.uint16, frame_shape[0] * frame_shape[1])
  _frame_lock = Lock()
  _running = Value(c_bool, False)

  comm_process = Process(target=run_async_loop, args=(_host, _port, _frame_shape, _frame_color, _frame_depth, _frame_lock,
                 _motor_values, _sensor_values, _num_sensors, camera, _running))
  comm_process.start()

def stop():
  global comm_process, _running
  _running.value = False
  time.sleep(0.3)
  comm_process.terminate()
  comm_process.join()

def sig_handler(signum, frame):
  if signum == signal.SIGINT:
    stop()
    sys.exit()

signal.signal(signal.SIGINT, sig_handler)

def set_frame(color: np.ndarray, depth: np.ndarray):
  global _frame_lock, _frame
  if color is None or depth is None: return
  _frame_lock.acquire()
  _frame = (color, depth)
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
  while running():
    print(read())
  stop()