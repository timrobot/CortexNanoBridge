#pragma config(UART_Usage, UART1, uartUserControl, baudRate115200, IOPins, None, None)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "uart_comms.c"

void ReadSensors()
{
  /** MODIFY THIS FUNCTION TO READ SENSORS **/
  // Example: SerialWriteSensor(quad1);
}

void RunMotors(int *val)
{
  motor[port1]  = val[0];
  motor[port2]  = val[1];
  motor[port3]  = val[2];
  motor[port4]  = val[3];
  motor[port5]  = val[4];
  motor[port6]  = val[5];
  motor[port7]  = val[6];
  motor[port8]  = val[7];
  motor[port9]  = val[8];
  motor[port10] = val[9];
}

task serialCommsTask()
{
  setTaskPriority(serialCommsTask, 20); // bump our priority
  while (true) {
    SerialUpdate();
    wait1Msec(2); // comms task expects to be run every 2mS
  }
}

task main()
{
  if (SerialInit(UART1, baudRate115200, "U1")) {
    startTask(serialCommsTask);
  }

  while (true) {
    // nothing to do here, event-based handling by other tasks
    wait1Msec(100);
  }
}
