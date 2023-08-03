from cortano import VexCortex, lan
from cortano import RealsenseCamera

if __name__ == "__main__":
  cam = RealsenseCamera()
  robot = VexCortex("/dev/ttyUSB0")
  lan.control(robot)
  lan.start("robot", frame_shape=(360, 640))

  while robot.running():
    color, depth = cam.read()
    lan.set_frame(color, depth)
    lan.check_alive()

  lan.stop()