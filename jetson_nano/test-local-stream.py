import cv2
from core import lan, overlord
from core.device import RealsenseCamera

if __name__ == "__main__":
  lan.start("0.0.0.0", 9990, (360, 640, 3), source=True)
  overlord.heartbeat()
  try:
    # cam = cv2.VideoCapture(0)
    cam = RealsenseCamera()
    while True:
      # _, frame = cam.read()
      color, depth = cam.read()
      lan.set_frame(color)
  except Exception as e:
    print(e)
  finally:
    lan.stop()