from flask import Flask, request
import json
from multiprocessing import Array, Value, Process
from ctypes import c_int, c_double
import time

app = Flask(__name__)

# shared variables
_motor_values = None
_sensor_values = None
_num_sensors = None
_last_rx_time = None

@app.route("/", methods=["GET"])
def index():
  return json.dumps({"status": "OK"})

@app.route("/update", methods=["POST"])
def update():
  global _motor_values, _sensor_values, _num_sensors, _last_rx_time

  motor_values = request.get_json()["motors"]
  if len(motor_values) == 10:
    _motor_values.acquire()
    _motor_values[:] = motor_values
    _motor_values.release()
    _last_rx_time.acquire()
    _last_rx_time.value = time.time()
    _last_rx_time.release()

  _sensor_values.acquire()
  nsensors = _num_sensors.value
  if nsensors == 0:
    sensor_values = []
  else:
    sensor_values = _sensor_values[:nsensors]
  _sensor_values.release()
  return json.dumps({
    "sensors": sensor_values
  })

def _runnable(_app: Flask, host, port, mval, sval, ns, rxt):
  global app, _motor_values, _sensor_values, _num_sensors, _last_rx_time
  app = _app
  _motor_values = mval
  _sensor_values = sval
  _num_sensors = ns
  _last_rx_time = rxt

  app.run(host=host, port=port)

worker: Process = None
controlled_entity = None

def set_passthrough(robot):
  global controlled_entity
  controlled_entity = robot

def start_rxtx(host, port):
  global app, _motor_values, _sensor_values, _num_sensors, _last_rx_time, worker
  global controlled_entity

  if controlled_entity is not None:
    _motor_values = controlled_entity._motor_values._data
    _sensor_values = controlled_entity._sensor_values._data
    _num_sensors = controlled_entity._num_sensors
  else:
    _motor_values = Array(c_int, 10)
    _sensor_values = Array(c_int, 20)
    _num_sensors = Value(c_int, 0)
  _last_rx_time = Value(c_double, time.time())

  worker = Process(target=_runnable, args=(
    app, host, port,
    _motor_values, _sensor_values, _num_sensors, _last_rx_time))
  worker.start()

def stop_rxtx():
  global worker
  worker.terminate()
  worker.join()