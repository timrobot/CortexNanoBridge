from core.robocomms import CortexController
import time

if __name__ == "__main__":
  robot = CortexController("COM4")
  robot.connect()
  
  while robot.running():
    robot.motor[1] = 0
    robot.motor[2] = 0
    robot.motor[3] = 0
    robot.motor[4] = 0
    time.sleep(0.01)