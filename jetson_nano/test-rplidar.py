from core.device import CortexController, RPLidar
from core import netcomms as nc

if __name__ == "__main__":
  robot = CortexController("COM4")
  robot.connect()
  nc.init(robot)

  lidar = RPLidar("COM5")

  while robot.running():
    if (scan := lidar.read()) is not None:
      pts = [list(pt) for pt in scan]
      nc.log(str(pts))

    keys = nc.keyboard()
    robot.motor[0] = (keys.KeyS - keys.KeyW) * 63
    robot.motor[9] = (keys.KeyQ - keys.KeyA) * 63

  nc.close()