from cortano import VexV5

if __name__ == "__main__":
  robot = VexV5() # or use VexCortex() for the older robot board

  while robot.running():
    sensors, battery = robot.sensors()
    # WARNING: Your robot starts moving, keep your surroundings clear.
    robot.motor[0] = 3
    robot.motor[9] = 0