#include "vex_comms.h"

int main(int argc, char *argv[]) {
  SerialInit(BAUD);

  while (1) {
    int *sensors = GetSensors();
    int pot = sensors[0];
    int low_lim = sensors[1];
    int high_lim = sensors[2];

    // scale speed from sensor value
    int speed = pot * 254 / 0xFFF - 127;

    // deadzone or limits
    if ((speed < 5 && speed > -5) ||
        (high_lim && speed > 0) ||
        (low_lim && speed < 0)) {
      speed = 0;
    }

    SetMotor(0, speed);
  }

  SerialDeInit();
}
