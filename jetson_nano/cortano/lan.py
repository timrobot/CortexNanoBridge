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
  Process,
  Lock,
  Manager,
  Array,
  RawArray,
  RawValue,
  Value
)
import ctypes

from cortano import rxtx

_frame_shape = (360, 640)
_frame = None
_frame_lock = threading.Lock()
_running = RawValue(ctypes.c_bool, False)
_connected = RawValue(ctypes.c_bool, False)
_encoding_parameters = [int(cv2.IMWRITE_PNG_COMPRESSION), 1]
_tx_ms_interval = .0125 # 80Hz

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
    frame = _frame
    _frame_lock.release()

    # frame = np.asarray(frame, np.uint8).reshape(shape)
    result, frame = cv2.imencode('.png', frame, _encoding_parameters) # changed
    data = pickle.dumps(frame, 0)
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

def _rxtx_worker(host, port, running: RawValue,
        rxbuf: RawArray, rxlen: RawValue, rxlock: Lock, rxtime: RawValue,
        txbuf: RawArray, txlen: RawValue, txlock: Lock, source):
  is_connected = False

  sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  if source:
    sock.bind((host, port))
    sock.listen()

  payload_size = struct.calcsize('>L')
  data = b""
  gathering_payload = True # states: gathering_payload, gathering_msg
  msg_size = 0
  txtime = time.time()

  while running.value:
    if not is_connected:
      data = b""
      gathering_payload = True
      try:
        ready_to_read, ready_to_write, in_error = select.select(
          [sock, ], [], [], 1
        )
        for s in ready_to_read:
          if s is sock:
            conn, address = sock.accept()
            is_connected = True

      except Exception as e:
        print("Warning:", e)
        continue
        
    if not is_connected: continue

    try:
      ready_to_read, ready_to_write, in_error = select.select(
        [conn,], [conn,], [], 2
      )
      if len(ready_to_read) > 0:
        received = conn.recv(4096)
        if received == b'':
          print("Warning: RxTx received empty bytes, closing and attempting reconnect...")
          conn.close()
          is_connected = False
          continue
        data += received
    except select.error:
      print("Warning: RxTx has been disconnected for 1.0 seconds, attempting reconnect...")
      conn.shutdown(2)
      conn.close()
      is_connected = False
      continue
    # retry connection...
    if not is_connected: continue

    curr_time = time.time()

    if gathering_payload:
      if len(data) >= payload_size:
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        gathering_payload = False
    else:
      if len(data) >= msg_size:
        rx = data[:msg_size]
        data = data[msg_size:]

        rx = pickle.loads(rx, fix_imports=True, encoding="bytes")
        rxlock.acquire()
        bytearr = rx.encode()
        rxlen.value = len(bytearr)
        rxbuf[:len(rx)] = bytearr
        rxtime.value = curr_time
        rxlock.release()

        gathering_payload = True

    if (curr_time - txtime) >= _tx_ms_interval:
      txtime = curr_time

      txlock.acquire()
      if txlen.value == 0:
        txlock.release() # do nothing
        continue
      tx = bytearray(txbuf[:txlen.value]).decode()
      txlock.release()

      tx = pickle.dumps(tx, 0)
      size = len(tx)

      try:
          conn.sendall(struct.pack('>L', size) + tx)
      except ConnectionResetError:
          is_connected = False
          conn.close()
      except ConnectionAbortedError:
          is_connected = False
          conn.close()
      except BrokenPipeError:
          is_connected = False
          conn.close()

  sock.close()

_host = "0.0.0.0"
_port = 9999

_tx_buf = RawArray(ctypes.c_uint8, 128)
_tx_len = RawValue(ctypes.c_int32, 0)
_tx_lock = Lock()
_rx_buf = RawArray(ctypes.c_uint8, 128)
_rx_len = RawValue(ctypes.c_int32, 0)
_rx_lock = Lock()

_rx_timestamp = RawValue(ctypes.c_float, 0.0)
_rx_tx_worker = None
_stream_thread = None

def start(host, port=9999, frame_shape=(360, 640)):
  global _frame_shape, _frame, _host, _port, _running, _stream_thread
  _host = "0.0.0.0"
  _port = port
  _frame_shape = frame_shape
  _frame = np.zeros((_frame_shape[0], _frame_shape[1] * 2, 3), np.uint8)

  if _running.value:
    print("Warning: stream is already running")
  else:
    _running.value = True
    _stream_thread = threading.Thread(target=_stream_sender, args=(_host, _port))

    # _rx_tx_worker = Process(target=_rxtx_worker, args=(
    #   _host, _port + 1, _running,
    #   _rx_buf, _rx_len, _rx_lock, _rx_timestamp,
    #   _tx_buf, _tx_len, _tx_lock, _source
    # ))

    _rx_tx_worker = Process(target=rxtx.app.run, args=( _host, _port + 1 ))
    _rx_tx_worker.start()
    _stream_thread.start()

def stop():
  global _running, _rx_tx_worker
  if _running.value:
    _running.value = False
    _rx_tx_worker.terminate()
    time.sleep(1)
    _rx_tx_worker.join()

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
  frame = np.concatenate((np.zeros_like(x1), x2.reshape(h, w, 1), x2.reshape(h, w, 1)), axis=-1)

  frame = np.concatenate((color, frame), axis=1)
  return frame

def set_frame(color: np.ndarray, depth: np.ndarray):
  global _frame_lock, _frame
  if color is None or depth is None: return
  frame = _encode_frame(color, depth)
  _frame_lock.acquire()
  _frame = frame
  _frame_lock.release()

def write(sensor_values, voltage_level=None):
  """Send motor values to remote location

  Args:
      sensor_values (List[int]): sensor values
      voltage_level (int, optional): Voltage of the robot. Defaults to None.
  """
  sensor_values = list(sensor_values)
  rxtx._write_lock.acquire()
  nsensors = rxtx._num_sensors.value = min(20, len(sensor_values))
  rxtx._sensor_values[:nsensors] = sensor_values
  rxtx._write_lock.release()

def read():
  """Return motor values that are read in

  Returns:
      List[int]: motor values
  """
  rxtx._read_lock.acquire()
  motor_values = rxtx._motor_values[:]
  rxtime = rxtx._last_rx_time.value
  rxtx._read_lock.release()
  if (time.time() - rxtime) > 1.0:
    motor_values = [0] * 10
  return motor_values
  
def recv():
  """**Deprecated.** Receive controls from socket

  Returns:
      str: message incoming from socket
  """
  global _rx_lock, _rx_buf, _rx_len, _rx_timestamp
  _rx_lock.acquire()
  if _rx_len.value == 0:
    _rx_lock.release()
    return None
  rx = bytearray(_rx_buf[:_rx_len.value])
  _rx_lock.release()
  msg = json.loads(rx.decode())

  # safety mechanism
  # if time.time() - _rx_timestamp.value > 0.5: # 500ms cutoff time
  #   msg = {
  #     "motor": [0] * 10
  #   }

  return msg

def send(msg):
  """**Deprecated.** Send a message through socket.

  Args:
      msg (str): message to send
  """
  global _tx_lock, _tx_buf, _tx_len
  _tx_lock.acquire()
  bytearr = json.dumps(msg).encode()
  _tx_buf[:len(bytearr)] = bytearr
  _tx_len.value = len(bytearr)
  _tx_lock.release()