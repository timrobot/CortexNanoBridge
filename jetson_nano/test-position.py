from core.device import Robot
from core import netcomms as nc

if __name__ == "__main__":
  robot = Robot("COM4", model="robot.json")
  nc.init(robot)

  while robot.running():
    x, y, yaw = nc.position("chassis")
    print(f"pose: {x}, {y}, {yaw}")

    keys = nc.keyboard()
    robot.motor[0] = (keys.KeyS - keys.KeyW) * 63
    robot.motor[9] = (keys.KeyQ - keys.KeyA) * 63

  nc.close()