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
import os

_frame_shape = (360, 640, 3)
_frame = None
_frame_lock = threading.Lock()

def _try_recv(sock, count=4096):
  try:
    ready_to_read, ready_to_write, in_error = select.select(
      [sock,], [sock,], [], 2
    )
    if len(ready_to_read) > 0:
      received = sock.recv(count)
      if received == b'':
        sock.close()
        print("Warning: stream received empty bytes, closing and attempting reconnect...")
        return -1, ready_to_read, ready_to_write, None
      return len(received), ready_to_read, ready_to_write, received
  except select.error:
    sock.shutdown(2)
    sock.close()
    print("Warning: stream has been disconnected for 1.0 seconds, attempting reconnect...")
    return -1, 0, 0, None

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
  connected.value = is_connected = False
  conn = None

  while running.value:
    if not is_connected:
      conn, address = sock.accept()
      connected.value = is_connected = True

    _frame_lock.acquire()
    frame = _frame
    _frame_lock.release()

    # frame = np.asarray(frame, np.uint8).reshape(shape)
    result, frame = cv2.imencode('.jpg', frame, encoding_params)
    data = pickle.dumps(frame, 0)
    size = len(data)

    try:
        conn.sendall(struct.pack('>L', size) + data)
    except ConnectionResetError:
        connected.value = is_connected = False
        conn.close()
        # running.value = False
    except ConnectionAbortedError:
        connected.value = is_connected = False
        conn.close()
        # running.value = False
    except BrokenPipeError:
        connected.value = is_connected = False
        conn.close()
        # running.value = False

  sock.close()

def _tx_worker(host, port,
        txbuf: RawArray, txlen: RawValue, txlock: Lock, txinterval,
        connected: RawValue, running: RawValue):
  tx_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  tx_socket.bind((host, port))
  tx_socket.listen()
  is_connected = False
  tx_conn = None
  last_tx_timestamp = time.time()

  while running.value:
    if not is_connected:
      tx_conn, address = tx_socket.accept()
      is_connected = True

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
          tx_conn.sendall(struct.pack('>L', size) + tx)
      except ConnectionResetError:
          is_connected = False
          tx_conn.close()
          # running.value = False
      except ConnectionAbortedError:
          is_connected = False
          tx_conn.close()
          # running.value = False
      except BrokenPipeError:
          is_connected = False
          tx_conn.close()
          # running.value = False

  tx_socket.close()

def _rx_worker(host, port,
        rxbuf: RawArray, rxlen: RawValue, rxlock: Lock, rxtime: RawValue,
        connected: RawValue, running: RawValue):
  rx_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  rx_socket.bind((host, port))
  rx_socket.listen()
  payload_size = struct.calcsize('>L')
  is_connected = False
  data = b""
  rx_conn = None

  while running.value:
    if not is_connected:
      rx_conn, address = rx_socket.accept()
      is_connected = True
      break

    while len(data) < payload_size:
      length, _, __, received = _try_recv(rx_conn)
      if length == -1:
        is_connected = False # attempt to reconnect
      elif length > 0:
        data += received
    # retry connection...
    if not is_connected: continue

    packed_msg_size = data[:payload_size]
    data = data[payload_size:]

    msg_size = struct.unpack(">L", packed_msg_size)[0]

    while len(data) < msg_size:
      length, _, __, received = _try_recv(rx_conn)
      if length == -1:
        is_connected = False # attempt to reconnect
      elif length > 0:
        data += received
    # retry connection...
    if not is_connected: continue

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

_botname = ""
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

def start(host=None, port=9999, frame_shape=(360, 640, 3)):
  global _frame_shape, _frame, _botname, _host, _port, _running, _stream_thread

  if host is None:
    host = "Unknown robot"
    config_path = f"/usr/local/cortano/robot-config.json"
    if os.path.exists(config_path):
      with open(config_path, "r") as fp:
        j = dict(json.load(fp))
        host = j.get("name", None)

  _botname = host
  _host = "0.0.0.0"
  _port = port
  _frame_shape = frame_shape
  _frame = np.zeros((_frame_shape), np.uint8)

  if _running.value:
    print("stream is already running")
  else:
    _running.value = True
    _stream_thread = threading.Thread(target=_stream_sender, args=(
      _host, _port,
      _frame, _frame_lock, _frame_shape, _encoding_parameters,
      _connected, _running
    ))

    # these are running under new processes since the threads are necessary for speed
    _processes.append(Process(target=_tx_worker, args=(
      _host, _port + 1,
      _tx_buf, _tx_len, _tx_lock, _tx_ms_interval,
      _connected, _running
    )))
    _processes.append(Process(target=_rx_worker, args=(
      _host, _port + 2,
      _rx_buf, _rx_len, _rx_lock, _rx_timestamp,
      _connected, _running
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