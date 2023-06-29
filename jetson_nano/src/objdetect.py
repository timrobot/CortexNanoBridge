# mobilenet v3 backbone instead of vgg16
# ssd-320 or
# mask rcnn

import torch
import torchvision
from torchvision import transforms

import numpy as np
from PIL import Image
import cv2
from multiprocessing import Process, Lock, RawArray, RawValue, Manager
import time
import ctypes

# from .device import RealsenseCamera
import assembly

from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation


def voxel_down_sample(point_cloud, radius): # 0.18ms on 640x360
  voxels = np.round(point_cloud / radius).astype(np.int32)
  point_cloud = np.unique(voxels, axis=0).astype(np.float32) * radius
  return point_cloud

def estimate_normals(pts, radius=1.0, max_nn=20):
  # https://pcl.readthedocs.io/projects/tutorials/en/latest/normal_estimation.html
  tree = KDTree(pts)
  nn_indices = tree.query_ball_point(pts, r=radius, return_sorted=True)
  normals = np.zeros_like(pts)
  for i in range(len(nn_indices)):
    # grab the min k nearest neighbors, normalize in order to do svd
    nearest_neighbors = pts[nn_indices[i]][:max_nn]
    if len(nearest_neighbors) < 3: continue # the normal is set to (0, 0, 0)
    nearest_neighbors -= nearest_neighbors.mean(axis=0).reshape((1, -1))
    u, s, vh = np.linalg.svd(nearest_neighbors, full_matrices=False)
    # normal vector is the last column
    normal = vh[:, 2]
    # try and see if the point @ normal is >0, if not, then flip it
    normals[i] = normal if normal @ pts[i] >= 0 else -normal
  return normals

def ICP_point2point(source, target, threshold, initial_transform,
                    fitness=1e-6, rmse=1e-6, max_iter=1):
  # http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html
  T = initial_transform
  for _ in range(max_iter):
    # find the closest points as correspondences
    # E(T) = SUM(|p - Tq|^2)
    projected = target @ T[:3,:3].T + T[:3,3:].T
    matched = KDTree(projected).query_ball_point(source, r=threshold, return_sorted=True)
    X, Y = [], []
    for i, x in enumerate(matched):
      if len(x) > 0:
        X.append(projected[x[0]])
        Y.append(source[i])
    X = np.array(X, np.float32)
    Y = np.array(Y, np.float32)
    n = X.shape[0]

    # once the closest point pairs are found, then recenter the points
    # https://www.youtube.com/watch?v=dhzLQfDBx2Q
    x0 = np.mean(X, axis=0)
    y0 = np.mean(Y, axis=0)
    X_ = X - x0.reshape((1, -1))
    Y_ = Y - y0.reshape((1, -1))

    # calculate the rotation
    H = X_.T @ Y_ / n
    u, s, vh = np.linalg.svd(H)
    R = vh.T @ u.T

    # calculate the translation
    t = y0.reshape((-1, 1)) - R @ x0.reshape((-1, 1))
    if np.sqrt(t.T @ t) < rmse:
      break
    
    T_ = np.eye(4)
    T_[:3,:3] = R
    T_[:3,3:] = t
    T = T_ @ T
  return T

def ICP_Point2Plane(source, target, source_normal, target_normal,
                    threshold, initial_transform,
                    fitness=1e-6, rmse=1e-6, max_iter=1):
  T = initial_transform

Point2Point = "ICP_Point2Point"
Point2Plane = "ICP_Point2Plane"

def estimate_pointcloud_transform(source, target, T=np.eye(4),
    voxel_radius=[0.016, 0.008, 0.004], iterations=[25, 15, 10],
    method=Point2Point):
  for radius, iteration in zip(voxel_radius, iterations):
    radius = 0.01
    A = voxel_down_sample(source, radius)
    B = voxel_down_sample(target, radius)

    An = estimate_normals(A, radius=radius * 2, max_nn=20)
    Bn = estimate_normals(B, radius=radius * 2, max_nn=20)

    if method == Point2Point:
      T = ICP_point2point(A, B, 0.5, T, max_iter=iteration)
    elif method == Point2Plane:
      T = ICP_point2point(A, B, An, Bn, T, max_iter=iteration)
  return T

class ObjectDetector:
  def __init__(self, camera, separate_process=False):
    self.camera = camera
    self.model = torchvision.models.detection.maskrcnn_resnet50_fpn(
      pretrained=True, pretrained_backbone=True)
    self.model.eval()
    self.model.to('cuda')
    
    height, width = self.camera.height, self.camera.width
    u = (np.arange(height, dtype=np.float32) - self.camera.cy) / self.camera.fy
    v = (np.arange(width,  dtype=np.float32) - self.camera.cx) / self.camera.fx
    u = np.tile(u.reshape((height, 1, 1)), (1, width, 1))
    v = np.tile(v.reshape((1, width, 1)), (height, 1, 1))
    self.uvf = np.concatenate((v, np.ones_like(u), -u), axis=-1)
    self.categories = [
        '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
        'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
        'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
        'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A', 'N/A',
        'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
        'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
        'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
        'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
        'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table',
        'N/A', 'N/A', 'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
        'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book',
        'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
      ]

    # models to compare to for the position
    self.mesh3 = {
      self.category2label("sports ball"): assembly.load_pts("tennis ball.stl")
    }
    self.diameter = 0.0
    self.pcd3 = {}

  def label2category(self, label):
    return self.categories[label]
  
  def category2label(self, category):
    if category in self.categories:
      return self.categories.index(category)
    return None

  def infer(self, color):
    preprocess = transforms.Compose([
      # transforms.Resize((320, 320)),
      transforms.ToTensor(),
      # transforms.Normalize(mean=[.485, .456, .406], std=[0.229, 0.224, 0.225])
    ])
    color = Image.fromarray(color)
    input_tensor = preprocess(color)
    input_batch = input_tensor.unsqueeze(0)
    input_batch = input_batch.to('cuda')
    with torch.no_grad():
      output = self.model(input_batch)[0]
    output = {l: output[l].to('cpu').numpy() for l in output}
    return output
  
  def detect(self, color, depth, category_filter=[], max_tolerance=0.0):
    assert(isinstance(category_filter, list))
    labels_filter = [self.category2label(category) for category in category_filter]
    output = self.infer(color)

    idx = 0
    while idx < output["labels"].shape[0]:
      if output["scores"][idx] < max_tolerance: break
      idx += 1

    if labels_filter: # we filter on labels
      filtered = []
      detected_labels = output["labels"][:idx]
      for i, label in enumerate(detected_labels):
        if label in labels_filter:
          filtered.append(i)
      if len(filtered) == 0: return []

      labels = output["labels"][filtered]
      scores = output["scores"][filtered]
      boxes = output["boxes"][filtered]
      masks = output["masks"][filtered]
    else:
      labels = output["labels"][:idx]
      scores = output["scores"][:idx]
      boxes = output["boxes"][:idx]
      masks = output["masks"][:idx]

    xyz = self.uvf * depth.reshape((-1,1))

    masks = masks.reshape((-1, color.shape[0], color.shape[1]))
    # pts3 = []
    # rgb3 = []
    T3 = []
    depth_mask = (depth.reshape((-1,)) > 0.0)
    # rgb = color.reshape((-1, 3))
    index = 0
    for label, mask in zip(labels, masks):
      idx = np.argwhere(np.logical_and(
        depth_mask, mask.reshape((-1,)) > 0.5)).reshape((-1,))
      # pts3.append(xyz[idx])
      pts = xyz[idx].copy()
      # rgb3.append(rgb[idx])
      # center the object to do mesh comparison
      obj_offset = np.mean(pts, axis=0)
      pts -= obj_offset # will have to add this in back later
      # minx = np.min(pts[:,0])
      # maxx = np.max(pts[:,0])
      # dia = maxx-minx
      np.save(f"{index}.npy", pts)
      index += 1
      # source_pcd = o3d.geometry.PointCloud()
      # source_pcd.points = o3d.utility.Vector3dVector(pts)
      # source_pcd.colors = o3d.utility.Vector3dVector(np.zeros_like(pts))
      # target_pcd = self.pcd3[label]
      T = estimate_pointcloud_transform(source_pcd, target_pcd)
      T3.append(T)

    # reformat the output
    output = []
    for item in zip(labels, scores, boxes, masks, T3):
      output.append(item)
    return output
  
if __name__ == "__main__":
  # fx = 460.92495728
  # fy = 460.85058594
  # cx = 315.10949707
  # cy = 176.72598267
  # height, width = 360, 640
  # source = np.zeros((height, width), dtype=np.float32)
  # target = np.zeros((height, width), dtype=np.float32)
  # # create a plane
  # plane = np.random.randint(low=0, high=20, size=(50, 50), dtype=np.int32).astype(np.float32) / 1000.
  # for i in range(50):
  #   for j in range(50):
  #     source[i + 100, j + 100] = plane[i, j] + 5.
  #     target[i + 100, j + 150] = plane[i, j] + 5.

  # u = (np.arange(height, dtype=np.float32) - cy) / fy
  # v = (np.arange(width,  dtype=np.float32) - cx) / fx
  # u = np.tile(u.reshape((height, 1, 1)), (1, width, 1))
  # v = np.tile(v.reshape((1, width, 1)), (height, 1, 1))
  # uvf = np.concatenate((v, np.ones_like(u), -u), axis=-1).reshape((-1, 3))

  # source_pcd = uvf * source.reshape((-1, 1))
  # target_pcd = uvf * target.reshape((-1, 1))

  source_pcd = voxel_down_sample(np.asarray(assembly.load_pts("tennis ball.stl").points), 0.001)
  target_pcd = voxel_down_sample(np.asarray(assembly.load_pts("tennis ball.stl").points) + \
      np.array([[0, 0, 0.25]], np.float32), 0.001)
  start_time = time.time()
  T = estimate_pointcloud_transform(source_pcd, target_pcd)
  print(time.time() - start_time)
  print(T)