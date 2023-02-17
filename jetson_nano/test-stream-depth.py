from core.device import CortexController, RealsenseCamera
from core import netcomms as nc
import numpy as np
import cv2

if __name__ == "__main__":
  robot = CortexController("COM4")
  robot.connect()
  nc.init(robot)

  cam = RealsenseCamera(640, 360)

  while robot.running():
    _, color, depth = cam.read("raw")
    depth_as_color = cv2.applyColorMap(
      np.sqrt(depth).astype(np.uint8), cv2.COLORMAP_HSV)
    nc.imshow(depth_as_color)

    keys = nc.keyboard()
    robot.motor[0] = (keys.KeyS - keys.KeyW) * 63
    robot.motor[9] = (keys.KeyQ - keys.KeyA) * 63

  nc.close()