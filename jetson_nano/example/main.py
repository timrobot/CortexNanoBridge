from cortano import VexV5, RealsenseCamera

if __name__ == "__main__":
  realsense = RealsenseCamera()
  robot = VexV5()

  while robot.running():
    color, depth = realsense.read()
    sensors, battery = robot.sensors()
    lefty = robot.controller.Axis3
    rightx = robot.controller.Axis1
    arm = robot.controller.ButtonR1 - robot.controller.ButtonR2
    claw = robot.controller.ButtonL1 - robot.controller.ButtonL2
    robot.motor[0] = (lefty + rightx) // 2
    robot.motor[9] = (lefty - rightx) // 2
    robot.motor[7] = arm * 50
    robot.motor[2] = claw * 50
