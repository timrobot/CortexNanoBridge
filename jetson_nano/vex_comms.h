#ifndef VEX_COMMS_H
#define VEX_COMMS_H

#define SER_PORT                 "/dev/ttyTHS1"
#define BAUD                     115200

int         SerialInit( long baud );
void        SerialDeInit();

int         *GetSensors();
void        SetMotor(int port, int value);

#endif
