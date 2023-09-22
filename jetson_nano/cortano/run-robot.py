from cortano import VexCortex
from cortano import RealsenseCamera

if __name__ == "__main__":
  # Note! To run this script locally on your robot, you must disable the nvcortexnano.service
  # Go to ../scripts and then run ./disable-autostart.sh
  cam = RealsenseCamera()
  robot = VexCortex("/dev/ttyUSB0")

  while robot.running():
    color, depth = cam.read()
    sensors = robot.sensors()
    robot.motor[0] = 0 # you can set this to any value from -127 to 127
    robot.motor[9] = 0