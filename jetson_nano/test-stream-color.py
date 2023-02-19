from core.device import Robot, RealsenseCamera
from core import netcomms as nc

if __name__ == "__main__":
  robot = Robot("COM4")
  nc.init(robot)

  # note: any camera can be used here, realsense is being used as an example
  cam = RealsenseCamera(640, 360)

  while robot.running():
    _, color, depth = cam.read()
    nc.imshow(color)

    keys = nc.keyboard()
    robot.motor[0] = (keys.KeyS - keys.KeyW) * 63
    robot.motor[9] = (keys.KeyQ - keys.KeyA) * 63

  nc.close()