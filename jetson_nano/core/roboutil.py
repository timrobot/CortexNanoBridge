import numpy as np
import time
import json
from multiprocessing import RawValue, Process
from ctypes import c_bool
from collections import deque
from . import vex_serial, assembly, device

def _run_virtual_robot(robot):
  last_time = time.time()
  while robot._keep_running.value:
    t = time.time()
    dt = t - last_time
    last_time = t

    robot.microcontroller(robot._motor_values, robot._sensor_values, dt) # same as map fn
    time.sleep(0.001)

class VirtualRobot:
  def __init__(self, model=None, microprocessor_loop=None):
    self._motor_values = vex_serial.IndexableArray(10)
    self._sensor_values = vex_serial.IndexableArray(20)
    self._keep_running = RawValue(c_bool, True)

    if model:
      with open(model, "r") as fp:
        self.description = json.load(fp)
        self.model = assembly.load(self.description)
    else:
      self.description = {}
      self.model = None

    device.Robot._entity = self # override entity

    self._worker = Process(target=_run_virtual_robot, args=(self,))
    self._worker.start()

  def running(self):
    return self._keep_running.value
  
  def microcontroller(self, motors, sensors, dt):
    pass

  @property
  def motor(self):
    return self._motor_values

  def motors(self):
    return self._motor_values.clone()

  @property
  def sensor(self):
    return self._sensor_values

  def sensors(self):
    return self._sensor_values.clone()
  
class DeviceHandler:
  def __init__(self, cacheResult=True):
    self._cached_readings = None
    self._cached_result = np.zeros((10,), dtype=np.double)
    self._cache_on = cacheResult
    self._timestamp = None

  def __call__(self):
    timestamp = device.Robot._entity.timestamp()
    if not self._cache_on or timestamp != self._timestamp: # dynamic sensor storage
      self._cached_result = device.Robot._entity.sensors()
      self._timestamp = timestamp
    return self._cached_result

  def value(self, other_sensors=None): # abstract function
    return self._cached_result

  def time(self):
    return self._timestamp

class SensorValue(DeviceHandler):
  def __init__(self, port: int):
    self.port = port

  def value(self, other_sensors=None):
    return device.Robot._entity.sensor[self.port]

class SensorEncoder(DeviceHandler):
  def __init__(self, outputRange: tuple, input: int,
      lower: int=None, upper: int=None, sensorRange: tuple=(-1e6, 1e6),
      memoryLength=3):
    super().__init__(True)
    self.outputRange = outputRange
    self.input = input
    self.lower = lower
    self.upper = upper
    self.sensorRange = sensorRange

    self.memory = deque(maxlen=memoryLength)
    self.vel = 0.0
    self.acc = 0.0

  def value(self, other_sensors=None):
    if len(self.memory) > 0 and \
        self.memory[-1][1] == device.Robot._entity.timestamp():
      return self.memory[-1][1]

    sensors = device.Robot._entity.sensors()
    value = sensors[self.input]
    if sensors[self.lower]: self.sensorRange = (value, self.sensorRange[1])
    if sensors[self.upper]: self.sensorRange = (self.sensorRange[0], value)

    v0, v1 = self.outputRange
    s0, s1 = self.sensorRange
    # detect for infinity, just in case input doesn't work
    if s0 == s1: return None
    value = float(value - s0) / float(s1 - s0) * float(v1 - v0) + v0

    self.memory.append((value, device.Robot._entity.timestamp()))
    return value

  def velocity(self):
    if len(self.memory) == 0 or \
        self.memory[-1][1] != device.Robot._entity.timestamp():
      self.value()
    
    vel = 0.0
    if len(self.memory) >= 2:
      ratio = 1.0 if len(self.memory) == 2 else 0.9
      for i in range(len(self.memory)-1):
        p0, t0 = self.memory[i]
        p1, t1 = self.memory[i+1]
        if t1 > t0:
          vel = (1.0 - ratio) * vel + ratio * (p1 - p0) / (t1 - t0)
    self.vel = vel
    return vel

  def acceleration(self):
    if len(self.memory) == 0 or \
        self.memory[-1][1] != device.Robot._entity.timestamp():
      self.value()

    acc = 0.0
    if len(self.memory) >= 3:
      ratio = 1.0 if len(self.memory) == 3 else 0.9
      for i in range(len(self.memory)-2):
        p0, t0 = self.memory[i]
        p1, t1 = self.memory[i+1]
        p2, t2 = self.memory[i+2]
        if t1 > t0 and t2 > t1:
          v1 = (p1 - p0) / (t1 - t0)
          v2 = (p2 - p1) / (t2 - t1)
          acc = (1.0 - ratio) * acc + ratio * (v2 - v1) / (t2 - t1)
    self.acc = acc
    return acc

class Motor:
  def __init__(self, port: int, max_acceleration=0.0, max_deceleration=0.0):
    self.port = port
    self.acceleration = max_acceleration
    self.deceleration = max_deceleration if max_deceleration != 0.0 else max_acceleration

    self.value = 0.0
    self._timestamp = 0.0

  def set(self, value):
    if self.acceleration == 0.0:
      self.value = value
    else:
      t = time.time()
      if self._timestamp == 0.0:
        self._timestamp = t
      dt = t - self._timestamp
      if dt > 1.0: dt = 1.0 # more than 1 second has passed >_<
      self._timestamp = t

      diff = value - self.value
      if (diff < 0 and value < 0) or (diff >= 0 and value >= 0):
        max_rate = dt * self.acceleration
      else:
        max_rate = dt * self.deceleration
      self.value += np.clip(diff, -max_rate, max_rate)

    device.Robot._entity.motor[self.port] = self.value

  def stop(self):
    self.value = 0.0
    device.Robot._entity.motor[self.port] = 0.0

class PIDController:
  # FIXME add max acceleration/deceleration for safety
  def __init__(self, port: int, input, kp=1.0, ki=0.0, kd=0.0,
      reverse=False, sumLimits=(-1.0, 1.0)):
    self.kp = kp
    self.ki = ki
    self.kd = kd

    self.reverse = reverse
    self.input = input
    self.port = port
    self.sumLimits = sumLimits

    self.sum_error = 0.0
    self.last_error = 0.0
    self._timestamp = None
    self.value = 0.0

  def set(self, value):
    if not self._timestamp:
      self._timestamp = time.time()
      return 0.0

    dt = time.time() - self._timestamp
    if dt > 0.0:
      input = self.input if not isinstance(self.input, DeviceHandler) else self.input()
      error = value - input

      self.sum_error = np.clip(self.sum_error + error * dt,
        self.sumLimits[0], self.sumLimits[1])
      d_error = (error - self.last_error) / dt

      self.value = np.clip(
        self.kp * error + \
        self.ki * self.sum_error + \
        self.kd * d_error, -127, 127)

      self.last_error = error
      if self.reverse: self.value = -self.value

    device.Robot._entity.motor[self.port] = self.value

  def reset(self):
    self.sum_error = 0.0
    self.last_error = 0.0
    self._timestamp = None
    self.value = 0.0

class StepRampValue:
  def __init__(self, start_value=0.0, range=(0.0, 1.0), ratio=1.0):
    self.value = start_value
    self.last_time = time.time()
    self.ratio = ratio
    self.range = range

  def __call__(self, _=None): # microcontroller method
    t = time.time()
    dt = t - self.last_time
    value = self.value + dt * self.ratio
    self.value = (value - self.range[0]) % (self.range[1] - self.range[0]) + self.range[0]
    self.last_time = t
    return self.value

class PendulumValue:
  def __init__(self, start_value=0.0, range=(0.0, 1.0), ratio=1.0):
    self.value = start_value
    self.last_time = time.time()
    self.ratio = ratio
    self.range = range

  def __call__(self, _=None):
    t = time.time()
    dt = t - self.last_time
    value = self.value + dt * self.ratio
    if value < self.range[0]:
      self.value = self.range[0] - (value - self.range[0])
      self.ratio = -self.ratio
    elif self.value > self.range[1]:
      self.value = self.range[1] - (value - self.range[1])
      self.ratio = -self.ratio
    else:
      self.value = value
    self.last_time = t
    return self.value