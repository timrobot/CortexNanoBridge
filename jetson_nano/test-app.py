from roboutil import (
  VirtualRobot,
  StepRampValue,
  PendulumValue
)
import netcomms as nc
from objects.atomics import (
  Point,
  Joint
)
import time
import cv2

class Robot(VirtualRobot):
  def __init__(self):
    super().__init__()
    # self.points = [
    #   Point((1, 0, 0)),
    #   Point((2, 0, 0)),
    #   Point((1, 1, 0)),
    #   Point((1, 0, 1))
    # ]
    # self.joints = [
    #   Joint(self.points[0], self.points[1], axis=(0, 0, 1)),
    #   Joint(self.points[0], self.points[2], axis=(1, 0, 0)),
    #   Joint(self.points[0], self.points[3], axis=(0, 1, 0))
    # ]
    # for joint in self.joints:
    #   joint.setInput(StepRampValue(0, (-180, 180), 360))

  def microcontroller(self, motors, sensors, dt):
    sensors[0] += motors[0] * dt
    sensors[1] += motors[1] * dt

if __name__ == "__main__":
  robot = Robot()
  # nc.render3d(robot.points[:2]) # just the first two mates for now
  robot.connect()
  
  # other stuff
  #cam = cv2.VideoCapture(0)
  motorval = PendulumValue(0, (-127, 127), 127)
  nc.log("Connected to robot")

  y = 0
  while robot.running():
    # nc.imshow(cam.read()[1])
    # print(nc.keyboard())
    nc.showBoundBoxes([("cube", (100, 100 + y, 50, 50))])
    robot.motor[0] = motorval()
    y = (y + 1) % 100
    time.sleep(0.01)