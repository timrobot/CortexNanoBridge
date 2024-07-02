from cortano import VexCortex, lan
from cortano import RealsenseCamera, getNextWebcamPath
import cv2

if __name__ == "__main__":
  realsense = RealsenseCamera()
  robot = VexCortex("/dev/ttyUSB0")
  lan.start(port=9999, robot=robot)
  second_cam_path = getNextWebcamPath()
  second_camera = None
  if second_cam_path is not None:
    second_camera = cv2.VideoCapture(second_cam_path)
    second_camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    second_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    second_camera.set(cv2.CAP_PROP_FPS, 30)

  try:
    while robot.running():
      lan.check_alive()
      color, depth = realsense.read()
      color2 = None
      if second_camera is not None:
        ret, color2 = second_camera.read()
      lan.set_frames(color, depth, color2)
  except KeyboardInterrupt:
    print("Keyboard interrupt received")
  finally:
    robot.stop()
    lan.stop()