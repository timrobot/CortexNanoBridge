import numpy as np
import threading
import time
import vex_serial
import netcomms as nc

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