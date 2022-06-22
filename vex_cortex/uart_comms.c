#ifndef UART_COMMS_C
#define UART_COMMS_C

// Code attribution source: https://github.com/jpearman/p3cortex

/*---------------------------------------------------------------------------*/
/*  START HEADER                                                             */
/*---------------------------------------------------------------------------*/

// function return values
#define SUCCESS      0
#define FAILURE      (-1)

// This can be 1 or 2 on the cortex
#ifndef MAX_PORTS
#define MAX_PORTS    1
#endif

#define MSGLEN     (128-2)

// Structure to hold message
typedef struct _message {
  unsigned char   cmd;
  unsigned char   length;
  unsigned char   data[MSGLEN];
} message_t;

#define BAUD                     baudRate115200

#define RX_BUF_SIZE              256
#define RX_NO_DATA               -1
#define RX_BUF_ERR               -2

// Fixed preambles for the messages (swapped for prefixes)
#define PREAMBLE1                0x50
#define PREAMBLE2                0xAF

// Defined commands (do not use A-F)
#define CMD_CONTROL_MOTOR_VALUES      ('M')
#define CMD_STATUS_SENSOR_VALUES      ('S')
#define CMD_STATUS_DEBUG              ('I')

// A structure to collect all information together for a single
// communications channel
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

} comms_t;


// Prototypes
int         SerialInit( int port, long baud=BAUD );
int         InitSerial();
void        InitMessage( message_t *message, unsigned char cmd, short data_size, short num_items );
int         SendMessage();
void        SerialWriteSensorValue(TSensorTypes _type, int _value);
int         ReceiveData();
int         DecodeMessage();
int         SerialUpdate();

// There are no function callbacks in ROBOTC, user must provide these functions
void      ReadSensors();
void      RunMotors(int *mtrV);

#define SerialWriteSensor(name) SerialWriteSensorValue(SensorType(name), SensorValue(name))

/*---------------------------------------------------------------------------*/
/*  END HEADER                                                               */
/*---------------------------------------------------------------------------*/

// No dynamic memory in ROBOTC so use fixed storage
static char *_prefix;
static comms_t MyComms;

// messages
static message_t _debug_message;
static message_t _motor_value_message;
static message_t _sensor_value_message;

#define MAX_SNSR_CNT  20
#define MAX_MTR_CNT   10

// read/write bufs
static int    _sensor_values[MAX_SNSR_CNT];
static char   _sensor_type[MAX_SNSR_CNT];
static int    _sensor_cnt;
static int    _motor_values[MAX_MTR_CNT];

#define ISHEX4(v)   (((v) >= '0' && (v) <= '9') || ((v) >= 'a' && (v) <= 'f'))
#define TOHEX4(v)   (((v) <= 9)   ? (v)+'0' : (v)+'a'-10)
#define FROMHEX4(v) (((v) <= '9') ? (v)-'0' : (v)-'a'+10)

/*---------------------------------------------------------------------------*/
/*  ROBOTC glue code                                                         */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  Peek to see if there are characters in the receive FIFO                  */
/*  There is no function to do this so we have to use getChar and store the  */
/*  received character id there is one                                       */
/*---------------------------------------------------------------------------*/

int
RobotC_PeekInput()
{
    if( MyComms.peekChar == (-1) )
        MyComms.peekChar = getChar(MyComms.port);

    return(MyComms.peekChar);
}

/*---------------------------------------------------------------------------*/
/*  Get next character from the receive FIFO                                 */
/*  return the character read using peek if that exists                      */
/*---------------------------------------------------------------------------*/

int
RobotC_GetChar()
{
    int c;

    if( MyComms.peekChar >= 0 )
        {
        c = MyComms.peekChar;
        MyComms.peekChar = -1;
        }
    else
        c = getChar(MyComms.port);

    return(c);
}

/*---------------------------------------------------------------------------*/
/*  Write buffer to the uart                                                 */
/*---------------------------------------------------------------------------*/

void
RobotC_WriteBuf( int port, char *data, int data_len )
{
    int     i;
    int     timeout;

    for(i=0;i<data_len;i++)
        {
        // wait for transmit complete or timeout
        // Under normal circumstances this should not be needed, there is
        // a transmit buffer larger than most messages
        // timeout here is set at about 200uS (from experimentation)
        timeout = 0;
        while( !bXmitComplete(port) && (timeout++ < 20) ) {}

        // send char
        sendChar(port, data[i]);
        }
}

/*---------------------------------------------------------------------------*/
/*  Wrapper function to set serial port parity                               */
/*---------------------------------------------------------------------------*/

enum serParType {
  SER_NO_PARITY,
  SER_ODD_PARITY,
  SER_EVEN_PARITY
};

void
RobotC_SetParity( int port, serParType parity )
{
    unsigned long *p;
    unsigned long  pc = 0;

#ifdef  _Target_Emulator_
    return;
#endif

    // port should be UART1 or UART2
    switch(port)
        {
        case    UART1:
            p = (unsigned long *)0x4000440C;
            break;
        case    UART2:
            p = (unsigned long *)0x4000480C;
            break;
        default:
            return;
        }

    // set the parity control bits
    // adjust data length to 9 bit when parity is on
    switch(parity)
        {
        case    SER_ODD_PARITY:
            pc = 0x00001600;
            break;
        case    SER_EVEN_PARITY:
            pc = 0x00001400;
            break;
        default:
            pc = 0;
            break;
        }

    // Set CR1
    *p = (*p & 0xFFFFE9FF) | pc;
}

/*---------------------------------------------------------------------------*/
/*  Init                                                                     */
/*---------------------------------------------------------------------------*/

int
SerialInit( int port, long baud, char *prefix ) {
  int i;
  _prefix = prefix;

  // Note which serial port we are using
  MyComms.port    = port;
  MyComms.baud    = baud;

  // no char waiting
  MyComms.peekChar = (-1);

  // zero motor and sensor values
  for (i = 0; i < 10; i++)
    _motor_values[i] = 0;

  for (i = 0; i < 20; i++)
    _sensor_values[i] = 0;

  // Create messge templates
  InitMessage( &_debug_message,        CMD_STATUS_DEBUG,         sizeof(char), MSGLEN );
  InitMessage( &_motor_value_message,  CMD_CONTROL_MOTOR_VALUES, sizeof(char), MSGLEN );
  InitMessage( &_sensor_value_message, CMD_STATUS_SENSOR_VALUES, sizeof(char), MSGLEN );

  // Init the serial port here
  InitSerial();

  return( true );
}

/*---------------------------------------------------------------------------*/
/*  Init Message                                                             */
/*---------------------------------------------------------------------------*/

void
InitMessage( message_t *message, unsigned char cmd, short data_size, short num_items ) {
  int i;

  message->cmd = cmd;
  message->length = data_size * num_items;
  for (i = 0; i < (int)message->length; i++) {
    message->data[i] = 0;
  }
}

/*---------------------------------------------------------------------------*/
/*      Initialize serial port                                               */
/*---------------------------------------------------------------------------*/

int
InitSerial()
{
    // Set baud before parity
    setBaudRate( MyComms.port, MyComms.baud );

    RobotC_SetParity( MyComms.port, SER_NO_PARITY );

    return(SUCCESS);
}
/*---------------------------------------------------------------------------*/
/*      Get sensor values and send them                                      */
/*---------------------------------------------------------------------------*/

void
SerialWriteSensorValue(TSensorTypes _type, int _value) {
  switch (_type) {
    case sensorAnalog:
    case sensorPotentiometer:
    case sensorReflection:
    case sensorLineFollower:
    case sensorGyro:
    case sensorAccelerometer:
      _sensor_type[_sensor_cnt] = 's';
      break;

    case sensorTouch:
    case sensorLEDtoVCC:
    case sensorDigitalIn:
    case sensorDigitalOut:
    case sensorDigitalHighImpedance:
      _sensor_type[_sensor_cnt] = 'w';
      break;

    case sensorSONAR_TwoPins_cm:
    case sensorSONAR_TwoPins_inch:
    case sensorSONAR_TwoPins_mm:
    case sensorSONAR_TwoPins_raw: // mm
      _sensor_type[_sensor_cnt] = 's';
      break;

    case sensorQuadEncoder:
      _sensor_type[_sensor_cnt] = 'l';
      break;

    default:
      _sensor_type[_sensor_cnt] = 'u';
      break;
  }

  _sensor_values[_sensor_cnt++] = _value;
}

#define EHEX(v) (((v) < 10) ? (v)+'0' : ((v) < 36 ? (v)+'A'-10 : (v)+'_'-36))

int
SendMessage() {
  int             i;
  char            *buf = &_sensor_value_message.data[0];
  char            *_data;
  int             _value;
  unsigned char   chk_sum = 0, total_bytes = 0;

  int             voltage_level = nAvgBatteryLevel;

  // We are going to have to use a special fmt to compose the message
  buf[0] = 0;  // in case of de-sync
  buf[1] = 0;  // in case of de-sync
  buf[2] = '[';
  buf[3] = CMD_STATUS_SENSOR_VALUES;
  _data = &buf[6];

  for (i = 0; i < _sensor_cnt; i++) {
    _value = (unsigned short)_sensor_values[i];

    *_data++ = _sensor_type[i];
    total_bytes++;
    switch (_sensor_type[i]) {
      case 'w': // 1 bit
        *_data++ = (_value & 0x1) + '0';
        total_bytes++;
        break;
      case 's': // 16 bits
        sprintf(_data, "%04x", _value & 0xFFFF);
        _data += 4;
        total_bytes += 4;
        break;
      case 'l': // 32 bits
        sprintf(_data, "%08x", _value);
        _data += 8;
        total_bytes += 8;
        break;
      default:
        *_data++ = _value;
        total_bytes++;
        break;
    }
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

  RobotC_WriteBuf(MyComms.port, buf, total_bytes + 3);
  return( SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Check for serial port for some data                                  */
/*---------------------------------------------------------------------------*/

int
ReceiveData() {
  int         c;
  int         delim_found = 0;

  // If we don't have recv any data, then just return
  if( RobotC_PeekInput() == RX_NO_DATA ) {
    return( RX_NO_DATA );
  }

  // We've recv'd something, so reset pointer first
  // initial partial packets will drop
  MyComms.rxcnt = 0;

  // Read everything available, we dont care about checksum
  do {
    c = RobotC_GetChar();
    if( c >= 0 ) {
      if (c == '[') {
        MyComms.rxcnt = 0;
      }
      MyComms.rxbuf[MyComms.rxcnt++] = c;
      if( MyComms.rxcnt == RX_BUF_SIZE )
        return( RX_BUF_ERR );
      if (c == ']') {
        delim_found = 1;
        memcpy(&_motor_value_message.data[0], &MyComms.rxbuf[0], MyComms.rxcnt);
        _motor_value_message.data[MyComms.rxcnt] = 0;
        _motor_value_message.length = MyComms.rxcnt;
        MyComms.rxcnt = 0;
        break;
      }
    }
  } while( c >= 0 );

  if (delim_found && _motor_value_message.data[0] == '[') {
    _motor_value_message.cmd = _motor_value_message.data[1];
    return( _motor_value_message.length );
  } else {
    return 0;
  }
 }

/*---------------------------------------------------------------------------*/
/*      Received some data so start to decode packet (format2)               */
/*---------------------------------------------------------------------------*/

#define EMBED_ERR_CODE(x) { \
    _motor_values[0] = (13 << 4) + 14 - 0x7F; \
    _motor_values[1] = (10 << 4) + 13 - 0x7F; \
    _motor_values[2] = x - 0x7F; \
}

int
DecodeMessage() {
  int             i, length;
  message_t       *msg = &_motor_value_message;
  char            m1, m2, *p = &msg->data[0];
  unsigned char   chk_sum, cmd;

  // message: [MLLDDDDDDDDDDDDDDDDDDDDCC]

  if (*p++ != '[' || msg->data[msg->length-1] != ']') {
    return( FAILURE );
  }

  cmd = *p++;
  if (cmd != CMD_CONTROL_MOTOR_VALUES) {
    return( FAILURE );
  }

  m1 = *p++;
  m2 = *p++;
  if (!ISHEX4(m1) || !ISHEX4(m2)) {
    return( FAILURE );
  }
  length = ((FROMHEX4(m1) << 4) + FROMHEX4(m2));
  if ((int)msg->length != length) {
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
  p = &msg->data[0];
  for (i = 0; i < length; i++) {
    chk_sum ^= *p++;
  }
  if (chk_sum != 0) {
    return( FAILURE );
  }

  p = &msg->data[4];
  for (i = 0; i < 10; i++) {
    m1 = *p++;
    m2 = *p++;

    if (!ISHEX4(m1) || !ISHEX4(m2)) {
      return( FAILURE );
    }
    _motor_values[i] = (int)((FROMHEX4(m1) << 4) + FROMHEX4(m2)) - 0x7F;
  }

  return( SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Call this task often for communications                              */
/*---------------------------------------------------------------------------*/

int
SerialUpdate() {
  int             rx_len;

  // Check for receive packet
  rx_len = ReceiveData();
  if( rx_len > 0 ) {
    DecodeMessage();
    RunMotors(_motor_values);
  }

  // Send out a packet no matter what (buffer overflow problems?)
  _sensor_cnt = 0;
  ReadSensors();
  if (MyComms.txto == 0) {
    SendMessage();
    MyComms.txto = 5; // reset the tx timeout to 10mS
  } else {
    MyComms.txto--;
  }

  return( SUCCESS );
}

#endif
