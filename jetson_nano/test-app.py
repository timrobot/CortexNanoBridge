from robocomms import RobotEntity
import netcomms as nc
import time
# import cv2

if __name__ == "__main__":
  robot = Robot("/dev/ttyUSB0")
  robot.connect()
  
  # other stuff which might be useful
  # cam = cv2.VideoCapture(0)
  nc.log("Connected to robot")

  while robot.running():
    # nc.imshow(cam.read()[1])
    # print(nc.keyboard())
    robot.motor[0] = 0
    time.sleep(0.01)
