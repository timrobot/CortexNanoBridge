import cv2
from core import lan, overlord

if __name__ == "__main__":
  lan.start("0.0.0.0", 9990, source=True)
  overlord.heartbeat()
  try:
    cam = cv2.VideoCapture(0)
    while True:
      _, frame = cam.read()
      lan.set_frame(frame)
  except:
    pass
  finally:
    lan.stop()