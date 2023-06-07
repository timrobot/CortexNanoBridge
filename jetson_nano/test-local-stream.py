import cv2
import lan

if __name__ == "__main__":
  lan.start("0.0.0.0", 9990, source=True) # 0.0.0.0 does not work, only localhost
  try:
    cam = cv2.VideoCapture(0)
    while True:
      _, frame = cam.read()
      lan.set_frame(frame)
  except:
    pass
  finally:
    lan.stop()