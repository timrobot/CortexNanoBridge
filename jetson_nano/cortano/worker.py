from cortano import VexCortex, lan_socket
from cortano import RealsenseCamera

if __name__ == "__main__":
  cam = RealsenseCamera(autostart=False)
  robot = VexCortex("/dev/ttyUSB0")
  lan_socket.start("robot", frame_shape=(360, 640), target=robot, camera=cam)

  while robot.running():
    lan_socket.check_alive()

  lan_socket.stop()