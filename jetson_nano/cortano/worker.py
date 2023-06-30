from cortano import VexCortex, lan
# from cortano import RealsenseCamera

if __name__ == "__main__":
  robot = VexCortex("/dev/ttyUSB0")
  lan.start("Unknown robot", frame_shape=(360, 1280, 3))
  # cam = RealsenseCamera()

  while robot.running():
    # frame = cam.get_combined_frame()
    # lan.set_frame(frame)

    msg = lan.recv()
    if msg and isinstance(msg, dict) and "motor" in msg:
      motor_values = msg["motor"]
      for i in range(10):
        robot.motor[i] = motor_values[i]
    lan.send({ "sensor": list(robot.sensors()) })
    # lan.heartbeat()

  lan.stop()