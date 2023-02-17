from core.device import CortexController
from core import netcomms as nc
import cv2

if __name__ == "__main__":
  robot = CortexController("COM4")
  robot.connect()
  nc.init(robot)

  cam = cv2.VideoCapture("/dev/video0")
  cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
  cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

  while robot.running():
    _, image = cam.read()
    nc.imshow(image)

    keys = nc.keyboard()
    robot.motor[0] = (keys.KeyS - keys.KeyW) * 63
    robot.motor[9] = (keys.KeyQ - keys.KeyA) * 63

  nc.close()