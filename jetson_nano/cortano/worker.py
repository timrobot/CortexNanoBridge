from cortano import VexCortex, lan
from cortano import RealsenseCamera

if __name__ == "__main__":
  cam = RealsenseCamera()
  robot = VexCortex("/dev/ttyUSB0")
  lan.start("robot", frame_shape=(360, 640))

  while robot.running():
    lan.set_frame(*cam.read())
    lan.write(robot.sensors())
    robot.motors(lan.read())

  lan.stop()