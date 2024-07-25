from cortano import VexCortex, RealsenseCamera, getNextWebcamPath
import cv2

if __name__ == "__main__":
  realsense = RealsenseCamera()
  robot = VexCortex("/dev/ttyUSB0") # or use VexV5()
  # secondCam = cv2.VideoCapture(getNextWebcamPath())

  while robot.running():
    color, depth = realsense.read()
    sensors, battery = robot.sensors()
    robot.motor[0] = 0 # you can set this to any value from -1 to 1
    robot.motor[9] = 0