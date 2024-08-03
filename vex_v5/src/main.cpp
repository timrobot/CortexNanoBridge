/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Wed Sep 25 2019                                           */
/*    Description:  Clawbot Template (Individual Motors)                      */
/*                                                                            */
/*    Name:                                                                   */
/*    Date:                                                                   */
/*    Class:                                                                  */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LeftMotor            motor         1               
// RightMotor           motor         10              
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>
#include <vector>

using namespace vex;

#define MATH_CLIP(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

void SetMotors(std::vector<int>& motor_values) {
  /**
   * User defined motors get set here!
   */

  char motor_val_str[128] = {0};
  sprintf(motor_val_str, "Motors: %d %d %d %d %d %d %d %d %d %d                                ",
    motor_values[0], motor_values[1], motor_values[2], motor_values[3], motor_values[4],
    motor_values[5], motor_values[6], motor_values[7], motor_values[8], motor_values[9]
  );
  Brain.Screen.printAt(10, 70, motor_val_str);

  int left = MATH_CLIP(motor_values[LeftMotor.index()], -100, 100);
  if (left < 0) {
    LeftMotor.spin(vex::directionType::rev, -left, vex::percentUnits::pct);
  } else if (left > 0) {
    LeftMotor.spin(vex::directionType::fwd, left, vex::percentUnits::pct);
  } else {
    LeftMotor.stop();
  }

  int right = MATH_CLIP(motor_values[RightMotor.index()], -100, 100);
  if (right < 0) {
    RightMotor.spin(vex::directionType::rev, -right, vex::percentUnits::pct);
  } else if (right > 0) {
    RightMotor.spin(vex::directionType::fwd, right, vex::percentUnits::pct);
  } else {
    RightMotor.stop();
  }

  int arm = MATH_CLIP(motor_values[ArmMotor.index()], -100, 100);
  if (arm < 0) {
    ArmMotor.spin(vex::directionType::rev, -arm, vex::percentUnits::pct);
  } else if (arm > 0) {
    ArmMotor.spin(vex::directionType::fwd, arm, vex::percentUnits::pct);
  } else {
    ArmMotor.stop();
  }

  int claw = MATH_CLIP(motor_values[ClawMotor.index()], -100, 100);
  if (claw < 0) {
    ClawMotor.spin(vex::directionType::rev, -claw, vex::percentUnits::pct);
  } else if (claw > 0) {
    ClawMotor.spin(vex::directionType::fwd, claw, vex::percentUnits::pct);
  } else {
    ClawMotor.stop();
  }
}

std::vector<short> ReadSensors() {
  std::vector<vex::motor> motors({LeftMotor, RightMotor, ArmMotor, ClawMotor});
  std::vector<short> sensor_values;
  for (vex::motor& m : motors) {
    short pos      = (short)m.position(vex::rotationUnits::deg);
    short vel      = (short)m.velocity(vex::velocityUnits::dps);
    short torque   = (short)(m.torque(vex::torqueUnits::Nm) * 1000);
    sensor_values.push_back(pos);
    sensor_values.push_back(vel);
    sensor_values.push_back(torque);
  }
  return sensor_values;
}


/*---------------------------------------------------------------------------*/
/*  Library code                                                             */
/*---------------------------------------------------------------------------*/

#define SUCCESS      0
#define FAILURE      (-1)

// Fixed preambles for the messages
#define PREAMBLE1                0x50
#define PREAMBLE2                0xAF

// Defined commands (do not use A-F)
#define CMD_CONTROL_MOTOR_VALUES      ('M')
#define CMD_STATUS_SENSOR_VALUES      ('S')
#define CMD_STATUS_DEBUG              ('I')
#define CMD_INFO_IP                   ('W')

#define MSGLEN     (256-2)

// Structure to hold message
typedef struct _message {
  unsigned char   cmd;
  unsigned char   length;
  unsigned char   data[MSGLEN];
} message_t;

#define RX_BUF_SIZE              (256+128)
#define RX_NO_DATA               -1
#define RX_BUF_ERR               -2

static message_t _motor_value_message;
static message_t _sensor_value_message;

#define MAX_SNSR_CNT  20
#define MAX_MTR_CNT   10 // legacy value, can be optimized in the future but not now

// read/write bufs
static std::vector<int> _motor_values(MAX_MTR_CNT, 0);
static std::vector<int> _motor_values_P(MAX_MTR_CNT, 0); // use P to reduce noise

#define ISHEX4(v)   (((v) >= '0' && (v) <= '9') || ((v) >= 'a' && (v) <= 'f'))
#define TOHEX4(v)   (((v) <= 9)   ? (v)+'0' : (v)+'a'-10)
#define FROMHEX4(v) (((v) <= '9') ? (v)-'0' : (v)-'a'+10)

typedef struct _comms {
    int             port;       // which Uart
    long            baud;       // baud rate of channel
    int             tcount;     // number of timeouts

    // Receive buffer
    int             rxcnt;                      // last amount of rx data
    unsigned char   rxbuf[RX_BUF_SIZE];         // buffer for rx data
    int             rxto;                       // timeout counter

    // Transmit control flow
    int             txto;

    // storage to help with checking rx buffer
    int             peekChar;

    // last rx time (safety check)
    struct time     lastrxtime;

} comms_t;

static comms_t MyComms;

int
ExtractMessage() {
  int     msg_found = 0;
  
  char *end = strrchr((char *)MyComms.rxbuf, ']');
  if (end != NULL) {
    char temp = end[1];
    end[1] = 0;
    char *start = strrchr((char *)MyComms.rxbuf, '[');
    if (start != NULL) {
      strcpy((char *)_motor_value_message.data, start);
      _motor_value_message.length = (int)(end - start + 1);
      end[1] = temp;
      MyComms.rxcnt -= (int)(end + 1 - (char *)MyComms.rxbuf);
      memmove((void *)MyComms.rxbuf, (void *)&end[1], MyComms.rxcnt);
      MyComms.rxbuf[MyComms.rxcnt] = 0;
      msg_found = 1;
    } else {
      end[1] = temp;
    }
  }

  if (MyComms.rxcnt > (256 + 32)) {
    memmove((void *)MyComms.rxbuf, (void *)&MyComms.rxbuf[64], MyComms.rxcnt - 64);
    MyComms.rxcnt -= 64;
    MyComms.rxbuf[MyComms.rxcnt] = 0;
  }

  if (msg_found && _motor_value_message.data[0] == '[') {
    _motor_value_message.cmd = _motor_value_message.data[1];
    return( _motor_value_message.length );
  } else {
    return 0;
  }
}

int
DecodeMessage() {
  int             i, length;
  message_t       &msg = _motor_value_message;
  char            m1, m2, *p = (char *)msg.data;
  unsigned char   chk_sum, cmd;

  // message: [MLLDDDDDDDDDDDDDDDDDDDDCC] or [WAABBCCDD]

  if (*p++ != '[' || msg.data[msg.length-1] != ']') {
    return( FAILURE );
  }

  cmd = *p++;
  if (cmd != CMD_CONTROL_MOTOR_VALUES && cmd != CMD_INFO_IP) {
    return( FAILURE );
  }

  if (cmd == CMD_INFO_IP && msg.length == 11) {
    char ipv4_str[50] = {0};
    sprintf(ipv4_str, "IPv4: %d.%d.%d.%d             ",
      (FROMHEX4(p[0]) << 4) + FROMHEX4(p[1]),
      (FROMHEX4(p[2]) << 4) + FROMHEX4(p[3]),
      (FROMHEX4(p[4]) << 4) + FROMHEX4(p[5]),
      (FROMHEX4(p[6]) << 4) + FROMHEX4(p[7])
    );
    Brain.Screen.printAt( 10, 50, ipv4_str );
    return (SUCCESS);
  }

  m1 = *p++;
  m2 = *p++;
  if (!ISHEX4(m1) || !ISHEX4(m2)) {
    return( FAILURE );
  }
  length = ((FROMHEX4(m1) << 4) + FROMHEX4(m2));
  if ((int)msg.length != length) {
    return( FAILURE );
  }

  p += length - 7; // 7 bytes for [MLL...CC]
  m1 = *p++;
  m2 = *p;
  if (!ISHEX4(m1) || !ISHEX4(m2)) {
    return( FAILURE );
  }
  chk_sum = ((FROMHEX4(m1) << 4) + FROMHEX4(m2));

  *p-- = 0;
  *p   = 0;
  p = (char *)msg.data;
  for (i = 0; i < length; i++) {
    chk_sum ^= *p++;
  }
  if (chk_sum != 0) {
    return( FAILURE );
  }

  p = (char *)&msg.data[4];
  for (i = 0; i < 10; i++) {
    m1 = *p++;
    m2 = *p++;

    if (!ISHEX4(m1) || !ISHEX4(m2)) {
      return( FAILURE );
    }
    _motor_values_P[i] = (int)((FROMHEX4(m1) << 4) + FROMHEX4(m2)) - 0x7F;
  }

  return( SUCCESS );
}


int receiveTask() {
    // enable port 18 as generic serial port
    vexGenericSerialEnable( vex::PORT19, 0 );
    // change baud rate, default is 230k
    vexGenericSerialBaudrate( vex::PORT19, 115200 );
    // allow vexos to reconfigure the port
    // the port will remain as a generic serial port until the brain is power cycled
    this_thread::sleep_for(10);

    vexGettime(&MyComms.lastrxtime);

    while(1) {
      // check to see if we have any bytes in the receive buffer
      int nRead = vexGenericSerialReceive( vex::PORT19,
        &MyComms.rxbuf[MyComms.rxcnt], RX_BUF_SIZE - MyComms.rxcnt - 1 );
    
      // if data found, then decode any message that may exist
      if( nRead > 0 ) {
        vexGettime(&MyComms.lastrxtime);
        MyComms.rxcnt += nRead;
        MyComms.rxbuf[MyComms.rxcnt] = 0;
        int res = ExtractMessage();
        if (res > 0) {
          DecodeMessage(); // set the motor values vector
        }
      }
    
      // read often
      this_thread::sleep_for(2);
    }
  
    return(0);
}

int
SendMessage(std::vector<short>& sensor_values) {
  int             i;
  char            *buf = (char *)_sensor_value_message.data;
  char            *_data;
  int             _value;
  unsigned char   chk_sum = 0, total_bytes = 0;

  int             voltage_level = (int)Brain.Battery.capacity(); // nImmediateBatteryLevel


  // We are going to have to use a special fmt to compose the message
  buf[0] = 0;  // in case of de-sync
  buf[1] = 0;  // in case of de-sync
  buf[2] = '[';
  buf[3] = CMD_STATUS_SENSOR_VALUES;
  _data = &buf[6]; // push to 6 since we need 4,5 for len

  // send the voltage level
  sprintf(_data, "%04x", voltage_level & 0xFFFF);
  _data += 4;
  total_bytes += 4;

  for (i = 0; i < sensor_values.size(); i++) {
    _value = (unsigned short)sensor_values[i];
    *_data++ = 's';
    sprintf(_data, "%04x", _value & 0xFFFF);
    _data += 4;
    total_bytes += 5;
  }

  total_bytes += 7; // [ Cmd Len1 Len2 ... ChkSum1 ChkSum2 ]
  buf[4] = TOHEX4((total_bytes >> 4) & 0xF);
  buf[5] = TOHEX4(total_bytes & 0xF);

  buf[total_bytes-1] = 0;
  buf[total_bytes] = 0;
  buf[total_bytes+1] = ']';
  buf[total_bytes+2] = '\n';

  for (i = 2; i < total_bytes + 2; i++) {
    chk_sum ^= buf[i];
  }
  buf[total_bytes-1] = TOHEX4((chk_sum >> 4) & 0xF);
  buf[total_bytes]   = TOHEX4(chk_sum & 0xF);

  vexGenericSerialTransmit( vex::PORT20, (uint8_t *)buf, total_bytes + 3 );
  return( SUCCESS );
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // start the rx thread
  vex::thread rxThread( receiveTask );

  // enable port 20 as generic serial port
  vexGenericSerialEnable( vex::PORT20, 0 );
  // change baud rate, default is 230k
  vexGenericSerialBaudrate( vex::PORT20, 115200 );
  // allow vexos to reconfigure the port
  // the port will remain as a generic serial port until the brain is power cycled
  this_thread::sleep_for(10);

  struct time currtime;

  while(1) {
    std::vector<short> sensor_values = ReadSensors();
    SendMessage(sensor_values);

    vexGettime(&currtime);
    int dti_hour = (int)currtime.ti_hour - (int)MyComms.lastrxtime.ti_hour;
    int dti_min  = (int)currtime.ti_min  - (int)MyComms.lastrxtime.ti_min;
    int dti_sec  = (int)currtime.ti_sec  - (int)MyComms.lastrxtime.ti_sec;
    int dti_hund = (int)currtime.ti_hund - (int)MyComms.lastrxtime.ti_hund;
    int dti_msec = ((((dti_hour * 60 + dti_min) * 60) + dti_sec) * 100 + dti_hund) * 10;
    if (dti_msec >= 1000) { // safety shutoff
      for (int i = 0; i < _motor_values.size(); i++) {
        _motor_values[i] = 0;
      }
    } else { // anti-noise ramp
      for (int i = 0; i < _motor_values.size(); i++) {
        _motor_values[i] += MATH_CLIP(_motor_values_P[i] - _motor_values[i], -4, 4);
      }
    }

    // set motor values here
    SetMotors(_motor_values);

    // Allow other tasks to run
    // send message every 10mS (100Hz)
    this_thread::sleep_for(10);
  }
}
