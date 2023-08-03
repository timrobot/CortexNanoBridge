from flask import Flask, request
import json
from multiprocessing import RawArray, Lock, RawValue, Process
from ctypes import c_int, c_double
import time

app = Flask(__name__)

# shared variables
_write_lock = None
_read_lock = None
_motor_values = None
_sensor_values = None
_num_sensors = None
_last_rx_time = None

@app.route("/", methods=["GET"])
def index():
  return json.dumps({"status": "OK"})

@app.route("/update", methods=["POST"])
def update():
  global _read_lock, _write_lock, _motor_values, _sensor_values, _num_sensors, _last_rx_time

  _read_lock.acquire()
  motor_values = request.get_json()["motors"]
  if len(motor_values) == 10:
    _motor_values[:] = motor_values
    _last_rx_time.value = time.time()
  _read_lock.release()

  _write_lock.acquire()
  nsensors = _num_sensors.value
  if nsensors == 0:
    sensor_values = []
  else:
    sensor_values = _sensor_values[:nsensors]
  _write_lock.release()
  return json.dumps({
    "sensors": sensor_values
  })

def _runnable(_app: Flask, host, port, wlock, rlock, mval, sval, ns, rxt):
  global app, _write_lock, _read_lock, _motor_values, _sensor_values, _num_sensors, _last_rx_time
  app = _app
  _write_lock = wlock
  _read_lock = rlock
  _motor_values = mval
  _sensor_values = sval
  _num_sensors = ns
  _last_rx_time = rxt

  app.run(host=host, port=port)

worker: Process = None

def start_rxtx(host, port):
  global app, _write_lock, _read_lock, _motor_values, _sensor_values, _num_sensors, _last_rx_time, worker

  _write_lock = Lock()
  _read_lock = Lock()
  _motor_values = RawArray(c_int, 10)
  _sensor_values = RawArray(c_int, 20)
  _num_sensors = RawValue(c_int, 0)
  _last_rx_time = RawValue(c_double, time.time())

  worker = Process(target=_runnable, args=(
    app, host, port,
    _write_lock, _read_lock, _motor_values, _sensor_values, _num_sensors, _last_rx_time))
  worker.start()

def stop_rxtx():
  global worker
  worker.terminate()
  worker.join()