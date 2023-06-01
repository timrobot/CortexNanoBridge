import numpy as np
from scipy.spatial.transform import Rotation
from typing import Dict, Tuple

################################################################################
## Links
################################################################################

class Link:
  _names = []

  def __init__(self, position=np.zeros((3,)), rotation=Rotation.identity(), name="Link"):
    self._position = np.array(position, dtype=np.float)
    self._rotation = rotation * Rotation.identity()
    self.setName(name)
    self.parent = None
    self.children = []

  def setName(self, name):
    name_count = 1
    if isinstance(name, tuple):
      name = name[0]
    while (name, name_count) in Link._names:
      name_count += 1
    self.name = (name, name_count)
    Link._names.append((name, name_count))

  def getName(self):
    return f"{self.name[0]} ({self.name[1]})"

  @property
  def x(self) -> float:               return self._position[0]
  @property
  def y(self) -> float:               return self._position[1]
  @property
  def z(self) -> float:               return self._position[2]
  @property
  def rotation(self) -> Rotation:     return self._rotation
  @property
  def position(self) -> np.ndarray:   return self._position.copy()

  def setPosition(self, position): # relative_to also includes orientation
    self._position = np.array(position, dtype=np.float)

  def setRotation(self, rotation: Rotation):
    self._rotation = rotation

  def setParent(self, parent):
    parent.addChild(self)
  
  def getElementByName(self, name):
    for child in self.children:
      if child.name == name:
        return child
      elif isinstance(child, Link):
        if (identified := child.getElementByName(name)) is not None:
          return identified
    return None

  def addChild(self, child):
    if child.parent:
      child.parent.removeChild(child)
    self.children.append(child)
    child.parent = self

  def removeChild(self, child):
    if child in self.children:
      self.children.remove(child)

  def copy(self):
    other = Link(self._position, self._rotation, self.name)
    for child in self.children:
      other.addChild(child.copy())
    return other

################################################################################
## Joints
################################################################################

class Joint(Link):
  def __init__(self, parent=None, child=None, name="Joint"):
    super().__init__(name=name)
    self.limits = None
    self.value = 0.0
    self.reverse = False

    if child is not None:
      self.addChild(child)
    if parent is not None:
      self.setParent(parent)

  def addChild(self, child): # override
    if child.parent:
      child.parent.removeChild(child)
    self.children = child
    child.parent = self

  def removeChild(self, child):
    self.children = None

class Revolute(Joint):
  def __init__(self, parent=None, child=None, axis=[1, 0, 0], name="Revolute"):
    super().__init__(parent, child, name)
    self.axis = np.array(axis, np.float)
    self.angle = 0.0

  @property
  def rotation(self):
    return Rotation.from_euler('zyx', [0, 0, self.angle], degrees=True)

class Linear(Joint):
  def __init__(self, parent=None, child=None, axis=[0, 0, 1], name="Linear"):
    super().__init__(parent, child, name)
    self.axis = np.array(axis, np.float)
    self.length = 0.0

  @property
  def position(self):
    return np.array([0, 0, self.length], np.float)

################################################################################
## Util
################################################################################

from stl import mesh
import open3d as o3d
import numpy as np

def load_pts(fname="Tennis Ball.stl"): # mm is unit -> 2.5mm voxel size -> meters convert
  m = mesh.Mesh.from_file(fname)
  vertices = m.points.reshape((-1, 3)) / 1000.
  pcd = o3d.geometry.PointCloud()
  pcd.points = o3d.utility.Vector3dVector(vertices)
  pcd.colors = o3d.utility.Vector3dVector(np.zeros_like(vertices))
  return pcd

def parseRotation(desc) -> Rotation:
  if "rpy" in desc:
    return Rotation.from_rotvec(desc["rpy"], degrees=True)
  elif "axis_angle" in desc:
    # http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
    axis = np.array(desc["axis_angle"][:3], dtype=np.float)
    angle = np.radians(float(desc["axis_angle"][3]))
    w = np.cos(angle / 2)
    s = np.sqrt(1 - w * w)
    if s > 1e-6: axis = axis * s
    return Rotation.from_quat(np.array([axis[0], axis[1], axis[2], w], dtype=np.float))
  else:
    return Rotation.identity()

def load(description: Dict) -> Link:
  root = Link(position=description.get("position", (0, 0, 0)),
              rotation=parseRotation(description),
              name=description.get("name", "robot"))
  
  links = {}
  for link_name, link in description["links"].items():
    links[link_name] = L = Link(name=link_name)
    if "xyz" in link:
      L.setPosition(link["xyz"])

  for link_name, link in description["links"].items():
    L = links[link_name]
    if "parent" in link:
      links[link["parent"]].add_child(L)
    else:
      root.addChild(L)

  joints = {}
  for joint_name, joint in description["joints"].items():
    if joint["type"] == "revolute":
      J = Revolute(name=joint_name)
    elif joint["type"] == "linear":
      J = Linear(name=joint_name)
    else:
      J = Joint(name=joint_name)
    joints[joint_name] = J
  
  for joint in description["joints"]:
    assert("parent" in joint)
    assert("child" in joint)

    J = Joint()
    if joint["parent"] in links.keys():
      parent = links[joint["parent"]]
    elif joint["parent"] in joints.keys():
      parent = joints[joint["parent"]]
    if joint["child"] in links.keys():
      child = links[joint["child"]]
    elif joint["child"] in joints.keys():
      child = joints[joint["child"]]

    J.addChild(child)
    J.setParent(parent)
  return root

def getJson(mate: Link):
  desc = {
    "name": "",
    "links": {},
    "joints": {}
  }

  def build_tree(root):
    if isinstance(root, Joint):
      assert(root.parent is not None)
      assert(root.children is not None)

      desc["joints"][root.name] = joint = {}
      joint["child"] = root.children.name
      joint["parent"] = root.parent.name

      if isinstance(root, Revolute):
        joint["type"] = "revolute"
      elif isinstance(root, Linear):
        joint["type"] = "linear"

      build_tree(root.children)
        
    elif isinstance(root, Link):
      desc["links"][root.name] = link = {
        "xyz": list(root.position),
        "rpy": list(root.rotation.as_rotvec(degrees=True))
      }
      if root.parent:
        link["parent"] = root.parent.name

      if root.children:
        for child in root.children:
          build_tree(child)

  build_tree(mate)
  return desc

def relativeTransform(target: Link, origin: Link=None) -> Tuple[Rotation, np.ndarray]:
  """
  Get relative position from two Links
  """
  # find root of subtrees
  curr = target
  target_parent_path = []
  while curr is not None and curr is not origin:
    target_parent_path.append(curr)

  origin_parent_path = []
  curr = origin
  while curr is not None and curr is not origin:
    origin_parent_path.append(curr)

  i = len(target_parent_path) - 1
  j = len(origin_parent_path) - 1
  while i >= 0 and j >= 0:
    if target_parent_path[i] != origin_parent_path[j]:
      break
    del target_parent_path[i]
    del origin_parent_path[j]

  Rot, tot = Rotation.identity(), np.zeros((3, 1), dtype=np.float)
  for curr in target_parent_path:
    Rcurr = curr.rotation
    Rot = Rcurr * Rot
    tot = Rcurr.apply(tot) + curr.position

  Rto, tto = Rotation.identity(), np.zeros((3, 1), dtype=np.float)
  for curr in origin_parent_path:
    Rcurr = curr.rotation
    Rto = Rcurr * Rto
    tto = Rcurr.apply(tto) + curr.position

  RtoI = Rto.inv()
  t = RtoI.apply(tot - tto)
  R = RtoI * Rot

  return R, t