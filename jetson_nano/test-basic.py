from core.device import CortexController

if __name__ == "__main__":
  robot = CortexController("COM4")
  robot.connect()
  while robot.running():
    robot.motor[0] = 0
    robot.motor[9] = 0