from ..cortano.device import RealsenseCamera
import cv2
import numpy as np

if __name__ == "__main__":
  cam = RealsenseCamera(640, 360)

  while True:
    color, depth = cam.read()
    depth_as_color = cv2.applyColorMap(np.sqrt(depth).astype(np.uint8), cv2.COLORMAP_HSV)
    # cv2.imshow("depth_as_color", depth_as_color)
    cv2.imshow("color", color)
    cv2.waitKey(1)