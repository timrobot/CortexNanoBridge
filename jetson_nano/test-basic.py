from core.device import Robot

if __name__ == "__main__":
  robot = Robot("COM4")
  while robot.running():
    robot.motor[0] = 0
    robot.motor[9] = 0