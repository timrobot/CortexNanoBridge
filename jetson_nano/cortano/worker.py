from cortano import VexCortex, lan
from cortano import RealsenseCamera, getNextWebcamPath
import cv2

if __name__ == "__main__":
  realsense = RealsenseCamera(autostart=False)
  robot = VexCortex("/dev/ttyUSB0")
  lan.start(port=9999, robot=robot, realsense=realsense, secondaryCam=False)
  second_cam_path = getNextWebcamPath()
  second_camera = None if second_cam_path is None else cv2.VideoCapture(second_cam_path)

  try:
    while robot.running():
      lan.check_alive()
      if second_camera is not None:
        ret, frame = second_camera.read()
        if ret:
          lan.set_secondary_frame(frame)
  except KeyboardInterrupt:
    print("Keyboard interrupt received")
  finally:
    robot.stop()
    lan.stop()