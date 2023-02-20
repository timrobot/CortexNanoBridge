import time
import os
ROOT = os.path.dirname(os.path.dirname(__file__))

import fractions
from typing import Tuple, List
import asyncio
import json
import ssl
import websockets
import requests

from aiohttp import web, helpers, web_runner
from av import VideoFrame
from av.frame import Frame
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription

import numpy as np
import ctypes
import json
import time
from multiprocessing import (
  Process,
  Lock,
  Manager,
  Array,
  RawArray,
  RawValue,
  Value
)

from . import assembly
from scipy.spatial.transform import Rotation

################################################################################
##    WebRTC
################################################################################

FPS = 30
VIDEO_CLOCK_RATE = 90000
VIDEO_PTIME = 1 / FPS
VIDEO_TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK_RATE)

class MediaStreamError(Exception):
  pass

class BufferVideoTrack(MediaStreamTrack):
  """
  A video stream track that gets from a sink
  """

  kind = "video"

  _start: float
  _timestamp: int

  def __init__(self, buf):
    super().__init__()  # don't forget this!
    self.buf = buf

  async def next_timestamp(self) -> Tuple[int, fractions.Fraction]:
    if self.readyState != "live":
      raise MediaStreamError

    if hasattr(self, "_timestamp"):
      self._timestamp += int(VIDEO_PTIME * VIDEO_CLOCK_RATE)
      wait = self._start + (self._timestamp / VIDEO_CLOCK_RATE) - time.time()
      await asyncio.sleep(wait)
    else:
      self._start = time.time()
      self._timestamp = 0
    return self._timestamp, VIDEO_TIME_BASE

  async def recv(self) -> Frame:
    # custom
    img = self.buf.data()

    if img is not None:
      frame = VideoFrame.from_ndarray(img, format="bgr24")

      pts, time_base = await self.next_timestamp()
      frame.pts = pts
      frame.time_base = time_base
    else:
      frame = None

    return frame

class DoubleFramebuffer:
  def __init__(self, width, height):
    self.height = height
    self.width = width
    self.bufs = [RawArray(ctypes.c_uint8, width * height * 3) for _ in range(2)]
    self.readidx = 0
    self.writeidx = 1
    self.lock = Lock()

    self.write_bufs = [np.frombuffer(buf, dtype=np.uint8)\
      .reshape((height, width, 3)) for buf in self.bufs]
    self.read_bufs = None

  def data(self):
    if self.read_bufs is None:
      self.read_bufs = [np.frombuffer(buf, dtype=np.uint8)\
        .reshape((self.height, self.width, 3)) for buf in self.bufs]

    self.lock.acquire()
    readidx = self.readidx
    buf = np.copy(self.read_bufs[readidx]).astype(np.uint8)
    self.lock.release()
    return buf

  def recv_frame(self, frame, format="rgb", fov=90):
    self.set(frame)

  def set(self, frame):
    if frame is not None and isinstance(frame, np.ndarray) and \
        frame.shape == (self.height, self.width, 3):
      self.lock.acquire()
      writeidx = self.writeidx
      self.lock.release()
      np.copyto(self.write_bufs[writeidx], frame)
      self.lock.acquire()
      self.writeidx, self.readidx = self.readidx, self.writeidx
      self.lock.release()

async def index(request):
    content = open(os.path.join(ROOT, "assets", "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)
    #  headers={"Feature-Policy": "gamepad 'self'"}) # this isnt supported yet

async def js(request):
    content = open(os.path.join(ROOT, "assets", "client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)
    #  headers={"Feature-Policy": "gamepad 'self'"}) # this isnt supported yet

last_time_of_update = None

class RTCStreamEntity:
  _peer_connections = set()
  rpc_calls = {}

  def __init__(self, bufEntity, host="0.0.0.0", stream_port=8080, req_port=9876,
      cert_file=None, key_file=None, loop=None):
    self.app = web.Application()
    self.app.on_shutdown.append(self.on_shutdown)

    self.bufferEntity = bufEntity
    self.app.router.add_get("/", index)
    self.app.router.add_get("/client.js", js)
    self.app.router.add_post("/offer", self.offer)
    
    self.host = host
    self.stream_port = stream_port
    self.req_port = req_port
    self.ssl_context = None
    if cert_file and key_file:
      self.ssl_context = ssl.SSLContext()
      self.ssl_context.load_cert_chain(cert_file, key_file)
    self.loop = loop
    self.created = False

    self.stream_task = None
    self.req_task = None

  async def offer(self, req) -> web.Response:
    params = await req.json()

    pc = RTCPeerConnection()
    RTCStreamEntity._peer_connections.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
      print("Connection state change is %s" % pc.connectionState)
      if pc.connectionState == "failed":
        await pc.close()
        RTCStreamEntity._peer_connections.discard(pc)

    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    await pc.setRemoteDescription(offer)
    pc.addTrack(BufferVideoTrack(self.bufferEntity))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
      content_type="application/json",
      text=json.dumps({
          "sdp":  pc.localDescription.sdp,
          "type": pc.localDescription.type
        })
    )

  async def request_handler(self, host, port):
    async with websockets.serve(self.recv_request, host, port):
      await asyncio.Future()

  async def recv_request(self, websocket, path):
    global last_time_of_update
    async for req in websocket:
      last_time_of_update.value = time.time()
      rpc = json.loads(req)
      full_res = {}
      if isinstance(rpc, str):
        rpc = { rpc: None }
      for fn in rpc.keys():
        if fn in RTCStreamEntity.rpc_calls.keys():
          args = rpc[fn]
          if args is not None:
            res = RTCStreamEntity.rpc_calls[fn](args)
          else:
            res = RTCStreamEntity.rpc_calls[fn]()
          if res:
            full_res[fn] = res

      def float_format(obj):
        if isinstance(obj, list):
          obj = [float_format(item) for item in obj]
        elif isinstance(obj, dict):
          obj = {key: float_format(obj[key]) for key in obj}
        elif isinstance(obj, float):
          obj = round(obj, 3)
        return obj
      await websocket.send(json.dumps(float_format(full_res)))

  async def on_shutdown(self, app):
    coros = [pc.close() for pc in RTCStreamEntity._peer_connections]
    await asyncio.gather(*coros)
    RTCStreamEntity._peer_connections.clear()

  def stop_streaming(self):
    web._cancel_tasks((self.stream_task, self.req_task), self.loop)
    web._cancel_tasks(helpers.all_tasks(self.loop), self.loop)

  def run(self):
    if self.loop is None:
      self.loop = asyncio.new_event_loop()
      self.created = True

    self.stream_task = self.loop.create_task(
      web._run_app(
        self.app,
        host=self.host,
        port=self.stream_port,
        ssl_context=self.ssl_context,
        access_log=None
      )
    )

    self.req_task = self.loop.create_task(
      self.request_handler(
        host=self.host,
        port=self.req_port
      )
    )

    if self.created:
      try:
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.stream_task)
      except (web_runner.GracefulExit, KeyboardInterrupt):
        pass
      finally:
        web._cancel_tasks((self.stream_task, self.req_task), self.loop)
        web._cancel_tasks(helpers.all_tasks(self.loop), self.loop)
        self.loop.run_until_complete(self.loop.shutdown_asyncgens())
        self.loop.close()

_rtc_entity = None # RTCStreamEntity

################################################################################
##    RPC Calls
################################################################################

_camera_buffer = None
TIME_TO_WAIT = 0.7

def connected():
  last_time_of_update.acquire()
  last_time = last_time_of_update.value
  last_time_of_update.release()
  return last_time > 0 and time.time() - last_time < TIME_TO_WAIT

LOGV = 4
LOGD = 3
LOGI = 2
LOGW = 1
LOGE = 0
MAX_LOG_LEN = 500

_log_buffer = None
_log_lock = None
main_process = None
main_manager = None
_global_robot = None

def log(msg: str, mode=LOGV):
  global _log_buffer, _log_lock, MAX_LOG_LEN
  _log_lock.acquire()
  if len(_log_buffer) > MAX_LOG_LEN:
    _log_buffer.pop(0)
  _log_buffer.append(msg)
  _log_lock.release()

# handler
def _rpc_logs():
  global _log_buffer, _log_lock
  _log_lock.acquire()
  buf = []
  while len(_log_buffer) > 0:
    buf.append(_log_buffer.pop(0))
  _log_lock.release()
  return buf

_enabled = False

def _rpc_enable():
  global _enabled
  _enabled.value = True

def _rpc_disable():
  global _enabled
  _enabled.value = False

_keyboard_keys = None
_keyboard_values = None

def _rpc_keydown(values):
  global _keyboard_keys, _keyboard_values
  _keyboard_values.acquire()
  for i in range(len(_keyboard_values)):
    _keyboard_values[i] = 0
  for keycode in values:
    if keycode in _keyboard_keys:
      idx = _keyboard_keys.index(keycode)
      _keyboard_values[idx] = 1
  _keyboard_values.release()

class KeyboardValues:
  def __init__(self, values=None):
    if values is None:
      values = [0] * 1000 # just something to get started

    keynames = [
      "ControlLeft",
      "ControlRight",
      "AltLeft",
      "AltRight",
      "ShiftLeft",
      "ShiftRight",
      "MetaLeft",
      "Backquote",
      "Minus",
      "Equal",
      "Backspace",
      # "Tab", # this causes errors
      "BracketLeft",
      "BracketRight",
      "Backslash",
      "CapsLock",
      "Semicolon",
      "Quote",
      "Enter",
      "Comma",
      "Period",
      "Slash",
      "Space",
      "ArrowUp",
      "ArrowDown",
      "ArrowLeft",
      "ArrowRight",
      "Home",
      "End",
      "PageDown",
      "PageUp",
      "Delete",
      "Insert"
    ]

    validx = 0
    for ch in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
      setattr(self, f"Key{ch}", values[validx])
      validx += 1
    for num in range(10):
      setattr(self, f"Digit{num}", values[validx])
      validx += 1
    for keyname in keynames:
      setattr(self, keyname, values[validx])
      validx += 1
    for fnkey in range(1, 13):
      setattr(self, f"F{fnkey}", values[validx])
      validx += 1

  def json(self):
    return self.__dict__

  def __str__(self):
    return json.dumps(self.json())

def keyboard() -> KeyboardValues:
  global _keyboard_keys, _keyboard_values
  _keyboard_values.acquire()
  values = KeyboardValues(_keyboard_values)
  _keyboard_values.release()
  return values

_gamepad_buttons = None
_gamepad_axes = None

def _rpc_gamepad(values):
  global _gamepad_buttons, _gamepad_axes
  buttons, axes = values["buttons"], values["axes"]
  _gamepad_buttons.acquire()
  for i in range(len(buttons)):
    _gamepad_buttons[i] = buttons[i]
  _gamepad_buttons.release()
  _gamepad_axes.acquire()
  for i in range(len(axes)):
    _gamepad_axes[i] = axes[i]
  _gamepad_axes.release()

class XboxGamepad:
  def __init__(self, buttons, axes):
    self.a      = buttons[0]
    self.b      = buttons[1]
    self.x      = buttons[2]
    self.y      = buttons[3]
    self.lb     = buttons[4]
    self.rb     = buttons[5]
    self.lt     = buttons[6]
    self.rt     = buttons[7]
    self.select = buttons[8]
    self.start  = buttons[9]
    self.ljoyz  = buttons[10]
    self.rjoyz  = buttons[11]
    self.up     = buttons[12]
    self.down   = buttons[13]
    self.left   = buttons[14]
    self.right  = buttons[15]
    self.ljoyx  = axes[0]
    self.ljoyy  = -axes[1]
    self.rjoyx  = axes[2]
    self.rjoyy  = -axes[3]

  def json(self):
    return self.__dict__

  def __str__(self):
    return json.dumps(self.json())

def gamepad() -> XboxGamepad:
  global _gamepad_buttons, _gamepad_axes
  _gamepad_buttons.acquire()
  _gamepad_axes.acquire()
  values = XboxGamepad(_gamepad_buttons, _gamepad_axes)
  _gamepad_buttons.release()
  _gamepad_axes.release()
  return values

def imshow(buf):
  global _camera_buffer
  _camera_buffer.set(buf)

_motors = None
_sensors = None
_robot_lock = None

def _rpc_robot_values():
  global _motors, _sensors, _robot_lock
  _robot_lock.acquire()
  motors = _motors[:] #[i for i in _motors]
  sensors = _sensors[:] #[i for i in _sensors]
  _robot_lock.release()
  return {
    "motors": motors,
    "sensors": sensors
  }

_bounding_boxes = None
_bblock = None

def showBoundBoxes(boundingBoxes: List):
  global _bounding_boxes, _bblock
  _bblock.acquire()
  _bounding_boxes[:] = boundingBoxes
  _bblock.release()

def _rpc_bounding_boxes():
  global _bounding_boxes, _bblock
  _bblock.acquire()
  bboxes = _bounding_boxes[:]
  _bblock.release()
  return bboxes

_mates_list = None
_mates_lock = None

def render3d(mates): # slow! try not to do this alot
  global _mates_list, _mates_lock
  _mates_lock.acquire()
  _mates_list[:] = mates
  _mates_lock.release()

def _rpc_obj3d():
  global _mates_list, _mates_lock
  _mates_lock.acquire()
  model = [assembly.getJson(mate) for mate in _mates_list]
  _mates_lock.release()
  return model

################################################################################
##    Positioning System
################################################################################

_cached_endpoint = None
def fiducialMarkers():
  global _cached_endpoint
  if _cached_endpoint is None:
    with open("config.json", "r") as fp:
      _cached_endpoint = json.load(fp)["endpoint"]
  res = requests.get(f'{_cached_endpoint}/get_tags').json()
  return res

_cached_links = None
_cached_assembly = None
_global_pose = None
def position(target=None):
  """
  Get position of the robot
  Args:
      gps (bool, optional): If False, use local fiducial system. Defaults to False.
  """
  global _cached_links, _cached_assembly
  if _cached_links is None:
    if not _global_robot \
        or "links" not in _global_robot.description \
        or len(_global_robot.description["links"].keys()) == 0:
      return None
    _cached_links = _global_robot.description["links"]
    _cached_assembly = _global_robot.model

  if target is None:
    target = _cached_links.values()[0]

  tag_info = fiducialMarkers()
  if len(tag_info) == 0:
    return None

  R_tags = []
  t_tags = []
  decisions = []
  for tag in tag_info:
    full_tag_name = f"{tag[0]}_{tag[1]}"
    if full_tag_name in _cached_links.keys():
      Rtag, ttag = assembly.relativeTransform(_cached_assembly,
        _cached_assembly.getElementByName(full_tag_name))
      Rcam, tcam = Rotation.from_rotvec(tag[3], degrees=True), tag[4]
      R_tags.append((Rcam * Rtag).as_rotvec(degrees=True))
      t_tags.append(Rcam.apply(ttag) + tcam)
      decisions.append(tag[2] / 100)

  # take the average pose
  weights = np.array(decisions, dtype=np.double) / np.sum(decisions)
  Ravg = Rotation.from_rotvec(R_tags, degrees=True).mean(weights)
  ypr = Ravg.as_euler('zyx', degrees=True)
  weights = np.tile(weights.reshape((-1, 1)), (1, 3))
  tavg = np.sum(weights * np.array(t_tags, dtype=np.double), axis=0)

  global _global_pose
  _global_pose[:] = [tavg[0], tavg[1], tavg[2], ypr[0], ypr[1], ypr[2]]

  return tavg[0], tavg[1], ypr[0]

def set_pose(x, y, yaw):
  global _global_pose
  _global_pose.acquire()
  _global_pose[:] = [x, y, 9, 0, 0, yaw]
  _global_pose.release()

def _rpc_pose():
  global _global_pose
  _global_pose.acquire()
  pose = _global_pose[:]
  _global_pose.release()
  return pose

################################################################################
##    Main Runnable
################################################################################

def _run_process(logbuf, lock, lasttime, en, cambuf, gpos, kboard, kvalues,
    gpbtns, gpaxes, bboxes, bblock, mlist, mlock, motors, sensors, rlock):
  RTCStreamEntity.rpc_calls["logs"]           = _rpc_logs
  RTCStreamEntity.rpc_calls["disable"]        = _rpc_disable
  RTCStreamEntity.rpc_calls["enable"]         = _rpc_enable
  RTCStreamEntity.rpc_calls["keydown"]        = _rpc_keydown
  # RTCStreamEntity.rpc_calls["gamepad"]        = _rpc_gamepad
  RTCStreamEntity.rpc_calls["robot_values"]   = _rpc_robot_values
  # RTCStreamEntity.rpc_calls["bounding_boxes"] = _rpc_bounding_boxes
  # RTCStreamEntity.rpc_calls["obj3d"]          = _rpc_obj3d
  RTCStreamEntity.rpc_calls["pose"]           = _rpc_pose

  global _log_buffer, _log_lock
  _log_buffer = logbuf
  _log_lock = lock

  global last_time_of_update
  last_time_of_update = lasttime

  global _enabled
  _enabled = en

  global _keyboard_keys, _keyboard_values
  _keyboard_keys = kboard
  _keyboard_values = kvalues

  global _gamepad_buttons, _gamepad_axes
  _gamepad_buttons = gpbtns
  _gamepad_axes = gpaxes

  global _bounding_boxes, _bblock
  _bounding_boxes = bboxes
  _bblock = bblock

  global _mates_list, _mates_lock
  _mates_list = mlist
  _mates_lock = mlock

  global _motors, _sensors, _robot_lock
  _motors = motors
  _sensors = sensors
  _robot_lock = rlock

  global _camera_buffer
  _camera_buffer = cambuf

  global _global_pose
  _global_pose = gpos

  global _rtc_entity
  _rtc_entity = RTCStreamEntity(_camera_buffer)
  _rtc_entity.run()

_initialized = False

def init(robot, frame_size=(640, 360)):
  global _initialized
  if _initialized:
    return
  _initialized = True

  global main_process, main_manager
  main_manager = Manager()

  global last_time_of_update
  last_time_of_update = Value(ctypes.c_double, 0)

  global _log_buffer, _log_lock
  _log_buffer = main_manager.list()
  _log_lock = Lock()

  global _enabled
  _enabled = RawValue(ctypes.c_bool, True)

  global _camera_buffer
  _camera_buffer = DoubleFramebuffer(*frame_size)

  global _global_pose
  _global_pose = Array(ctypes.c_double, 6)

  global _keyboard_keys, _keyboard_values
  _keyboard_keys = KeyboardValues().json().keys()
  _keyboard_values = Array(ctypes.c_int, len(_keyboard_keys))

  global _gamepad_buttons, _gamepad_axes
  _gamepad_buttons = Array(ctypes.c_double, 17)
  _gamepad_axes = Array(ctypes.c_double, 4)

  global _bounding_boxes, _bblock
  _bounding_boxes = main_manager.list()
  _bblock = Lock()

  global _mates_list, _mates_lock
  ml = main_manager.list()
  if _mates_list:
    for item in _mates_list:
      ml.append(item)
  _mates_list = ml
  _mates_lock = Lock()

  global _global_robot
  global _motors, _sensors, _robot_lock
  _global_robot = robot
  _motors = _global_robot._motor_values._data
  _sensors = _global_robot._sensor_values._data
  _robot_lock = Lock()

  if hasattr(robot, "render3d"):
    render3d(robot.render3d())

  main_process = Process(target=_run_process, args=(
    _log_buffer, _log_lock,
    last_time_of_update,
    _enabled,
    _camera_buffer,
    _global_pose,
    _keyboard_keys, _keyboard_values,
    _gamepad_buttons, _gamepad_axes,
    _bounding_boxes, _bblock,
    _mates_list, _mates_lock,
    _motors, _sensors, _robot_lock
  ))
  main_process.start()

def close():
  global main_process
  main_process.terminate()
  main_process.join(3)
  if main_process.is_alive():
    main_process.kill()
  main_process.close()