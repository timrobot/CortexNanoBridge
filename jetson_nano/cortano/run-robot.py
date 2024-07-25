from cortano import VexV5, VexCortex, RealsenseCamera

if __name__ == "__main__":
  realsense = RealsenseCamera()
  robot = VexV5() # or use VexCortex() for the older robot board

  while robot.running():
    color, depth = realsense.read()
    sensors, battery = robot.sensors()
    robot.motor[0] = 0 # you can set this to any value from -100 to 100
    robot.motor[9] = 0