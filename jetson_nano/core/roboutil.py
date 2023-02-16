import numpy as np
import threading
import time
from collections import deque
from . import vex_serial
from . import netcomms as nc
from . import robocomms

class VirtualRobot(threading.Thread):
  def __init__(self):
    super().__init__()
    self._motor_values = vex_serial.ProtectedZeroList(10)
    self._sensor_values = vex_serial.ProtectedZeroList(20)
    self.last_time = time.time()
    self._running = False

  def run(self):
    while True:
      t = time.time()
      dt = t - self.last_time
      self.last_time = t
      motors = self.motors()
      sensors = self._sensor_values

      self.microcontroller(motors, sensors, dt) # same as map fn
      time.sleep(0.001)

  def microcontroller(self, motors, sensors, dt):
    """
    Use this function in order to simulate what might happen on the
    microcontroller (eg. sensor value changes)
    It can be as simple as direct sensor manipulation or as complicated as a
    simulator
    """
    pass

  def connect(self):
    self._running = True
    nc.init(self)
    self.start()

  def running(self):
    return self._running

  def onUpdate(self, handler):
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
    self._cached_result = 0.0
    self._cache_on = cacheResult
    self._timestamp = None

  def __call__(self):
    state = robocomms.CortexController._entity.state()
    if not self._cache_on or state.timestamp != self._timestamp: # dynamic sensor storage
      self._cached_result = self.value(state)
      self._timestamp = state.timestamp
    return self._cached_result

  def value(self, _): # abstract function
    return self._cached_result

  def time(self):
    return self._timestamp

class SensorValue(DeviceHandler): # source state
  def __init__(self, port: int):
    self.port = port

  def value(self, state: robocomms.State):
    return state.sensors[self.port] # we want to update the internal state as well

class SensorEncoder(DeviceHandler):
  def __init__(self, outputRange: tuple, input: int,
      lower: int=None, upper: int=None, sensorRange: tuple=(-1e6, 1e6),
      memoryLength=2):
    self.outputRange = outputRange
    self.input = input
    self.lower = lower
    self.upper = upper
    self.sensorRange = sensorRange

    self.memory = deque(maxlen=memoryLength)
    self.vel = 0.0
    self.acc = 0.0

  def value(self, state: robocomms.State):
    value = state.sensors[self.input]
    if state.sensors[self.lower]: self.sensorRange = (value, self.sensorRange[1])
    if state.sensors[self.upper]: self.sensorRange = (self.sensorRange[0], value)

    v0, v1 = self.outputRange
    s0, s1 = self.sensorRange
    # detect for infinity, just in case input doesn't work
    if s0 == s1: return None
    value = float(value - s0) / float(s1 - s0) * float(v1 - v0) + v0

    self.memory.append((value, state.timestamp))
    return value

  def velocity(self):
    vel = 0.0
    if len(self.memory) >= 2:
      ratio = 1.0 if len(self.memory) == 2 else 0.9
      for i in range(len(self.memory)-1):
        a1, t1 = self.memory[i+1]
        a0, t0 = self.memory[i]
        if t1 > t0:
          vel = (1.0 - ratio) * vel + ratio * (a1 - a0) / (t1 - t0)
    self.vel = vel
    return vel

  def acceleration(self):
    acc = 0.0
    if len(self.memory) >= 3:
      ratio = 1.0 if len(self.memory) == 3 else 0.9
      for i in range(len(self.memory)-2):
        a2, t2 = self.memory[i+2]
        a1, t1 = self.memory[i+1]
        a0, t0 = self.memory[i]
        if t1 > t0 and t2 > t1:
          v1 = (a1 - a0) / (t1 - t0)
          v2 = (a2 - a1) / (t2 - t1)
          acc = (1.0 - ratio) * acc + ratio * (v2 - v1) / (t2 - t1)
    self.acc = acc
    return acc

class Motor:
  def __init__(self, port: int, max_acceleration=0.0, max_deceleration=0.0):
    self.port = port
    self.acceleration = max_acceleration
    self.deceleration = max_deceleration if max_deceleration != 0.0 else max_acceleration

    self.current = 0.0
    self._timestamp = None

  def set(self, value):
    if self.acceleration == 0.0:
      self.current = value
    else:
      t = time.time()
      if self._timestamp is None:
        self._timestamp = t
      dt = t - self._timestamp
      if dt > 1.0: dt = 1.0 # more than 1 second has passed >_<
      self._timestamp = t

      diff = value - self.current
      if (diff < 0 and value < 0) or (diff >= 0 and value >= 0):
        max_rate = dt * self.acceleration
      else:
        max_rate = dt * self.deceleration
      self.current += np.clip(diff, -max_rate, max_rate)

    robocomms.CortexController._entity.motor[self.port] = self.current

  def stop(self):
    self.current = 0.0
    robocomms.CortexController._entity.motor[self.port] = 0.0

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
    self.current = 0.0

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

      self.current = np.clip(
        self.kp * error + \
        self.ki * self.sum_error + \
        self.kd * d_error, -127, 127)

      self.last_error = error
      if self.reverse: self.current = -self.current

    robocomms.CortexController._entity.motor[self.port] = self.current

  def reset(self):
    self.sum_error = 0.0
    self.last_error = 0.0
    self._timestamp = None
    self.current = 0.0

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