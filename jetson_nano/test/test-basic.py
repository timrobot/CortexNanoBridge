from ..cortano.vex_serial import VexCortex

if __name__ == "__main__":
  robot = VexCortex("COM4")
  while robot.running():
    robot.motor[0] = 0
    robot.motor[9] = 0