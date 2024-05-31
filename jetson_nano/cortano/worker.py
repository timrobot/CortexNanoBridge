from cortano import VexCortex, lan
from cortano import RealsenseCamera

if __name__ == "__main__":
  cam = RealsenseCamera(autostart=False)
  robot = VexCortex("/dev/ttyUSB0")
  lan.start(port=9999, frame_shape=(360, 640), robot=robot, realsense=cam)

  while robot.running():
    lan.check_alive()

  lan.stop()