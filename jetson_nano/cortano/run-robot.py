from cortano import VexCortex, RealsenseCamera, getNextWebcamPath
import cv2

if __name__ == "__main__":
  # Note! To run this script locally on your robot, you must disable the nvcortexnano.service
  # Go to ../scripts and then run ./disable-autostart.sh
  realsense = RealsenseCamera()
  robot = VexCortex("/dev/ttyUSB0")
  secondCam = cv2.VideoCapture(getNextWebcamPath())

  while robot.running():
    color, depth = realsense.read()
    sensors, voltage = robot.sensors()
    robot.motor[0] = 0 # you can set this to any value from -127 to 127
    robot.motor[9] = 0