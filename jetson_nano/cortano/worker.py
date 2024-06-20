from cortano import VexCortex, lan
from cortano import RealsenseCamera

if __name__ == "__main__":
  realsense = RealsenseCamera(autostart=False)
  robot = VexCortex("/dev/ttyUSB0")
  lan.start(port=9999, robot=robot, realsense=realsense, secondaryCam=True)

  while robot.running():
    lan.check_alive()

  lan.stop()