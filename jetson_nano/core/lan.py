import cv2
import numpy as np

import socket
import pickle
import struct
import threading
import json
import time
import requests

from multiprocessing import (
  Process,
  Lock,
  Manager,
  Array,
  RawArray,
  RawValue,
  Value
)
import ctypes

OVERLORD_IP = "http://192.168.1.34:3000"
_frame_shape = (360, 640, 3)
_frame = None
_frame_lock = threading.Lock()


def _stream_sender(host, port,
        buf: RawArray, lock: Lock, shape, encoding_params,
        connected: RawValue, running: RawValue):
  """
  Streams data out
  """
  global _frame, _frame_lock, _frame_shape
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.bind((host, port))
  sock.listen()
  while running.value:
    connection, address = sock.accept()
    connected.value = True
    break

  while running.value:
    _frame_lock.acquire()
    frame = _frame
    _frame_lock.release()

    # frame = np.asarray(frame, np.uint8).reshape(shape)
    result, frame = cv2.imencode('.jpg', frame, encoding_params)
    data = pickle.dumps(frame, 0)
    size = len(data)

    try:
        connection.sendall(struct.pack('>L', size) + data)
    except ConnectionResetError:
        running.value = False
    except ConnectionAbortedError:
        running.value = False
    except BrokenPipeError:
        running.value = False

  sock.close()
            
def _stream_receiver(host, port,
        buf: RawArray, lock: Lock,
        connected: RawValue, running: RawValue):
  """
  Handles the connection and processes its stream data.
  """
  global _frame, _frame_lock, _frame_shape
  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  sock.connect((host, port))
  connected.value = True
  payload_size = struct.calcsize('>L')
  data = b""

  while running.value:
    break_loop = False

    while len(data) < payload_size:
      received = sock.recv(4096)
      if received == b'':
        sock.close()
        break_loop = True
        break
      data += received

    if break_loop:
      break

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]

    msg_size = struct.unpack(">L", packed_msg_size)[0]

    while len(data) < msg_size:
      data += sock.recv(4096)

    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
    _frame_lock.acquire()
    _frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)#.reshape((-1,))
    _frame_lock.release()

  sock.close()

def _tx_worker(host, port,
        txbuf: RawArray, txlen: RawValue, txlock: Lock, txinterval,
        connected: RawValue, running: RawValue, source):
  tx_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  if source:
    tx_socket.bind((host, port))
    tx_socket.listen()
    while running.value:
      tx_pipe, address = tx_socket.accept()
      connected.value = True
      break
  else:
    tx_socket.connect((host, port))
    tx_pipe = tx_socket

  last_tx_timestamp = time.time()

  while running.value:
    curr_time = time.time()
    if curr_time - last_tx_timestamp >= txinterval:
      last_tx_timestamp = curr_time

      txlock.acquire()
      if txlen.value == 0:
        txlock.release() # do nothing
        continue
      tx = bytearray(txbuf[:txlen.value]).decode()
      txlock.release()

      tx = pickle.dumps(tx, 0)
      size = len(tx)

      try:
          tx_pipe.sendall(struct.pack('>L', size) + tx)
      except ConnectionResetError:
          running.value = False
      except ConnectionAbortedError:
          running.value = False
      except BrokenPipeError:
          running.value = False

  tx_socket.close()

def _rx_worker(host, port,
        rxbuf: RawArray, rxlen: RawValue, rxlock: Lock, rxtime: RawValue,
        connected: RawValue, running: RawValue, source):
  rx_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  if source:
    rx_socket.bind((host, port))
    rx_socket.listen()
    while running.value:
      rx_pipe, address = rx_socket.accept()
      connected.value = True
      break
  else:
    rx_socket.connect((host, port))
    rx_pipe = rx_socket

  payload_size = struct.calcsize('>L')
  data = b""

  while running.value:
    break_loop = False

    while len(data) < payload_size:
      received = rx_pipe.recv(4096)
      if received == b'':
        rx_pipe.close()
        break_loop = True
        break
      data += received

    if break_loop: # transmission failed
      break

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]

    msg_size = struct.unpack(">L", packed_msg_size)[0]

    while len(data) < msg_size:
      data += rx_pipe.recv(4096)

    rx = data[:msg_size]
    data = data[msg_size:]

    rx = pickle.loads(rx, fix_imports=True, encoding="bytes")
    rxlock.acquire()
    bytearr = rx.encode()
    rxlen.value = len(bytearr)
    rxbuf[:len(rx)] = bytearr
    rxtime.value = time.time()
    rxlock.release()

  rx_socket.close()

_host = "0.0.0.0"
_port = 9999
_encoding_parameters = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
_tx_ms_interval = .02 # 50Hz

_running = RawValue(ctypes.c_bool, False)
_connected = RawValue(ctypes.c_bool, False)

_tx_buf = RawArray(ctypes.c_uint8, 128)
_tx_len = RawValue(ctypes.c_int32, 0)
_tx_lock = Lock()
_rx_buf = RawArray(ctypes.c_uint8, 128)
_rx_len = RawValue(ctypes.c_int32, 0)
_rx_lock = Lock()

_rx_timestamp = RawValue(ctypes.c_float, 0.0)
_processes = []
_stream_thread = None

def query_overlord(hostname):
  global OVERLORD_IP
  for i in range(300): # try for 300 seconds
    res = requests.get(f"{OVERLORD_IP}/get-robots").json()
    for robot in res:
      if robot["name"] == hostname:
        return robot["ipv4"]
    time.sleep(1)

def publish_to_overlord(hostname):
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  s.settimeout(0)
  s.connect(('10.254.254.254', 1))
  IP = s.getsockname()[0]
  
  requests.post(f"{OVERLORD_IP}/heartbeat",
    json={"name": hostname, "ipv4": IP})

def start(host, port=9999, frame_shape=(360, 640, 3), source=False):
  global _frame_shape, _frame, _host, _port, _running, _stream_thread
  if source:
    publish_to_overlord(host)
    _host = "0.0.0.0"
  else:
    if not "." in host:
      _host = query_overlord(host)
    else:
      _host = host
  
  _port = port
  _frame_shape = frame_shape
  _frame = np.zeros((_frame_shape), np.uint8)

  if _running.value:
    print("stream is already running")
  else:
    _running.value = True
    if source:
      _stream_thread = threading.Thread(target=_stream_sender, args=(
        _host, _port,
        _frame, _frame_lock, _frame_shape, _encoding_parameters,
        _connected, _running
      ))
    else:
      _stream_thread = threading.Thread(target=_stream_receiver, args=(
        _host, _port,
        _frame, _frame_lock,
        _connected, _running
      ))

    _processes.append(Process(target=_tx_worker, args=(
      _host, _port + (1 if source else 2),
      _tx_buf, _tx_len, _tx_lock, _tx_ms_interval,
      _connected, _running, source
    )))
    _processes.append(Process(target=_rx_worker, args=(
      _host, _port + (2 if source else 1),
      _rx_buf, _rx_len, _rx_lock, _rx_timestamp,
      _connected, _running, source
    )))

    _stream_thread.start()
    for p in _processes:
      p.start()

def stop():
  global _running, _processes
  if _running.value:
    _running.value = False
    time.sleep(1)
    for p in _processes:
      p.kill()

def set_frame(frame: np.ndarray):
  global _frame_lock, _frame
  _frame_lock.acquire()
  _frame = frame
  _frame_lock.release()
  
def get_frame():
  global _frame_lock, _frame
  _frame_lock.acquire()
  frame = _frame
  _frame_lock.release()
  return frame

def recv():
  global _rx_lock, _rx_buf, _rx_len
  _rx_lock.acquire()
  if _rx_len.value == 0:
    _rx_lock.release()
    return None
  rx = bytearray(_rx_buf[:_rx_len.value])
  _rx_lock.release()
  msg = json.loads(rx.decode())
  return msg

def send(msg):
  global _tx_lock, _tx_buf, _tx_len
  _tx_lock.acquire()
  bytearr = json.dumps(msg).encode()
  _tx_buf[:len(bytearr)] = bytearr
  _tx_len.value = len(bytearr)
  _tx_lock.release()