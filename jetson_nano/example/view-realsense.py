from cortano import RealsenseCamera
import cv2
import numpy as np

if __name__ == "__main__":
  realsense = RealsenseCamera()

  while True:
    color, depth = realsense.read()
    depth_as_color = np.sqrt(depth).astype(np.uint8)
    cv2.imshow('depth', depth)
    cv2.imshow('color', color)
    cv2.waitKey(1)
    print((color, depth))