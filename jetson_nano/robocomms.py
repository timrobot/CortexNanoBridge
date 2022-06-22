import vex_serial

import time
from collections import deque
import numpy as np
import netcomms as nc

_entity = None

class State:
  def __init__(self, sensorValues, timestamp=None):
    self.sensors = sensorValues
    self.timestamp = timestamp if timestamp else time.time()
    self.values = {}

class RobotEntity(vex_serial.VexCortex): # just a wrapper really with state mgmt
  def __init__(self, prefix=""):
    super().__init__()
    self.prefix = prefix #FIXME need to do something with this
    self._state = State(self.sensors(), 0.0)

    global _entity
    if not _entity:
      _entity = self

  def state(self) -> State:
    # make sure we are always up to date
    updateable = False
    if self._last_rx_timestamp:
      updateable = not self._state.timestamp or \
        self._state.timestamp < self.timestamp() # yes, we are calling "twice" (as shown below)

    if updateable:
      self._rx_timestamp_lock.acquire()
      state = State(self.sensors(), self._last_rx_timestamp)
      self._rx_timestamp_lock.release()
      self._state = state

    return self._state

  def obs(self) -> np.ndarray:
    return None

  def act(self, _: np.ndarray):
    pass

  def connect(self):
    global _entity
    if _entity is self:
      nc.init(self)

    self.start()

  def disconnect(self):
    self.close()
    self.join()

  def running(self):
    return self.is_alive()

class DeviceHandler:
  def __init__(self, cacheResult=True):
    self._cached_readings = None
    self._cached_result = 0.0
    self._cache_on = cacheResult
    self._timestamp = None

  def __call__(self):
    global _entity
    state = _entity.state()
    if not self._cache_on or state.timestamp != self._timestamp: # dynamic sensor storage
      self._cached_result = self.value(state)
      self._timestamp = state.timestamp
    return self._cached_result

  def value(self, _): # abstract function
    return self._cached_result

  def time(self):
    return self._timestamp

class SensorValue(DeviceHandler): # source state
  def __init__(self, index: int):
    self.index = index

  def value(self, state: State):
    return state.sensors[self.index] # we want to update the internal state as well

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

  def value(self, state: State):
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

class MotorController:
  def __init__(self, index: int, max_acceleration=0.0, max_deceleration=0.0):
    self.index = index
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

    global _entity
    _entity.motor[self.index] = self.current

  def stop(self):
    self.current = 0.0
    _entity.motor[self.index] = 0.0

class PIDMotorController:
  # FIXME add max acceleration/deceleration for safety
  def __init__(self, index: int, input, kp=1.0, ki=0.0, kd=0.0,
      reverse=False, sumLimits=(-1.0, 1.0)):
    self.kp = kp
    self.ki = ki
    self.kd = kd

    self.reverse = reverse
    self.input = input
    self.index = index
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

    global _entity
    _entity.motor[self.index] = self.current

  def reset(self):
    self.sum_error = 0.0
    self.last_error = 0.0
    self._timestamp = None
    self.current = 0.0