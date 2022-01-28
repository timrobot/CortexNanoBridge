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

// Structure to hold transmit or receive packet
typedef struct _packet {
  unsigned char   data[sizeof(message_t) + 3]; // P1, P2, cmd, len, data, chksum
  short           msg_cnt;
  short           msg_len;
  unsigned char   chk_sum;
} packet_t;

#define BAUD                     baudRate115200

#define RX_BUF_SIZE              256
#define RX_NO_DATA               -1
#define RX_BUF_ERR               -2

// Fixed preambles for the messages
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

    // Transmit and Receive packets
    packet_t        TxPak;
    packet_t        RxPak;

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
int         SendDebugMessage( char *data );
int         SendFmt2SensorMessage();
void        UARTWriteSensorValue(TSensorTypes _type, short _value);
int         SendMessage( message_t *message );
int         SendPacket( packet_t *pkt );
message_t * GetMessageStructure( unsigned char cmd );
int         ReceiveData();
int         ReceiveFmt2Packet();
void        ReceivePacket();
void        DecodeMessage( message_t *message );
int         SerialUpdate();

// There are no function callbacks in ROBOTC, user must provide these functions
void      ReadSensors();
void      RunMotors(int *mtrV);

#define SerialWriteSensor(name) UARTWriteSensorValue(SensorType(name), SensorValue(name))

/*---------------------------------------------------------------------------*/
/*  END HEADER                                                               */
/*---------------------------------------------------------------------------*/

// No dynamic memory in ROBOTC so use fixed storage
static comms_t MyComms;

// messages
static message_t _debug_message100;
static message_t _motor_value_message;
static message_t _sensor_value_message;

#define MAX_SNSR_CNT  20
#define MAX_MTR_CNT   10

// read/write bufs
static short  _sensor_values[MAX_SNSR_CNT];
static char   _sensor_type[MAX_SNSR_CNT];
static int    _sensor_cnt;
static int    _motor_values[MAX_MTR_CNT];
static char   _extra_buf[256];
static int    _extra_buf_len;

#define ISHEX4(v)   (((v) >= '0' && (v) <= '9') || ((v) >= 'A' && (v) <= 'F'))
#define TOHEX4(v)   (((v) < 10)   ? (v)+'0' : (v)+'A'-10)
#define FROMHEX4(v) (((v) <= '9') ? (v)-'0' : (v)-'A'+10)

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
SerialInit( int port, long baud ) {
  int i;

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
  InitMessage( &_debug_message100,  CMD_STATUS_DEBUG, sizeof(char), 100 );
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
/*      Send a message as debug line                                         */
/*---------------------------------------------------------------------------*/

int
SendDebugMessage(char *data) {
  int    strlength = 0;
  char   *p, *q;
  int    i;

  while (data[strlength] > 0) {
    strlength++;
  }
  if (strlength > 100) {
    return( FAILURE );
  }

  _debug_message100.length = strlength + 2;
  p = data;
  q = &_debug_message100.data[0];
  *q++ = 0;
  *q++ = 0;
  if (strlength > 0) {
    for (i = 0; i < strlength; i++) {
      *q++ = *p++;
    }
  }

  RobotC_WriteBuf( MyComms.port, _debug_message100.data, _debug_message100.length );

  //SendMessage( &_debug_message100 );

  return( SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Get sensor values and send them                                      */
/*---------------------------------------------------------------------------*/

void
UARTWriteSensorValue(TSensorTypes _type, short _value) {
  switch (_type) {
    case sensorAnalog:
    case sensorPotentiometer:
    case sensorReflection:
    case sensorLineFollower:
    case sensorGyro:
    case sensorAccelerometer:
      _sensor_type[_sensor_cnt] = 'a';
      break;

    case sensorTouch:
    case sensorLEDtoVCC:
    case sensorDigitalIn:
    case sensorDigitalOut:
    case sensorDigitalHighImpedance:
      _sensor_type[_sensor_cnt] = 'd';
      break;

    case sensorSONAR_TwoPins_cm:
    case sensorSONAR_TwoPins_inch:
    case sensorSONAR_TwoPins_mm:
    case sensorSONAR_TwoPins_raw: // mm
      _sensor_type[_sensor_cnt] = 's';
      break;

    case sensorQuadEncoder:
      _sensor_type[_sensor_cnt] = 'q';
      break;

    default:
      _sensor_type[_sensor_cnt] = 'u';
      break;
  }

  _sensor_values[_sensor_cnt++] = _value;
}

#define EHEX(v) (((v) < 10) ? (v)+'0' : ((v) < 36 ? (v)+'A'-10 : (v)+'_'-36))

int
SendFmt2SensorMessage() {
  int             i, j;
  char            *buf = &_sensor_value_message.data[0];
  int             total_bytes = 0, num_bytes;
  char            *_data;
  short           _value;
  unsigned char   chk_sum = 0, v;

  // reset sensor id
  _sensor_cnt = 0;
  ReadSensors();

  // We are going to have to use a special fmt to compose the message
  buf[0] = 0;  // in case of de-sync
  buf[1] = 0;  // in case of de-sync
  buf[2] = '[';
  buf[3] = CMD_STATUS_SENSOR_VALUES;
  _data = &buf[6];

  for (i = 0; i < _sensor_cnt; i++) {
    _value = _sensor_values[i];

    switch (_sensor_type[i]) {
      case 'd': // 1 bit
        num_bytes = 1;
        break;
      case 'a': // 12 bits
        num_bytes = 3;
        break;
      case 's': // 12 bits
        num_bytes = 3;
        break;
      case 'q': // 16 bits
        num_bytes = 4;
        break;
      default:
        num_bytes = 2;
        break;
    }
    total_bytes += 1 + num_bytes;

    // write to buf
    *_data++ = _sensor_type[i];
    _data += num_bytes;
    for (j = 0; j < num_bytes; j++) {
      v = _value & 0xF;
      *(--_data) = TOHEX4(v);
      _value >>= 4;
    }
    _data += num_bytes;
  }

  total_bytes += 8; // [ Cmd Len1 Len2 ... ChkSum1 ChkSum2 ] \n
  v = total_bytes & 0xF;
  buf[5] = TOHEX4(v);
  v = (total_bytes >> 4) & 0xF;
  buf[4] = TOHEX4(v);

  buf[total_bytes-2] = 0;
  buf[total_bytes-1] = 0;
  buf[total_bytes] = ']';
  buf[total_bytes+1] = '\n';

  for (i = 2; i < 2 + total_bytes; i++) {
    chk_sum ^= buf[i];
  }
  v = chk_sum & 0xF;
  buf[total_bytes-1] = TOHEX4(v);
  v = (chk_sum >> 4) & 0xF;
  buf[total_bytes-2] = TOHEX4(v);

  RobotC_WriteBuf(MyComms.port, buf, total_bytes + 2);

  return( SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Take message and place into tx packet                                */
/*---------------------------------------------------------------------------*/

int
SendMessage( message_t *message ) // SendPacket()
{
  unsigned int    i;
  unsigned char   *p, *q;
  packet_t        *pkt;

  pkt = &MyComms.TxPak;

  // Get command length, add 5 bytes for overhead
  // [PR1|PR2|CMD|LEN|data|CHK_SUM]
  pkt->msg_len = (message->length) + 5;

  // Create header
  pkt->data[0] = 0;
  pkt->data[1] = 0; // for some reason we need these two
  pkt->data[2] = PREAMBLE1;
  pkt->data[3] = PREAMBLE2;
  pkt->data[4] = message->cmd;
  pkt->data[5] = message->length;

  // Start of checksum
  q = &pkt->data[2];
  i = 0;
  for (pkt->chk_sum = 0; i < 4; i++)
    pkt->chk_sum ^= *q++;

  // move any data that exists
  if (message->length > 0) {
    p = &message->data[0];
    for (i = 0; i < message->length; i++) {
      pkt->chk_sum ^= *p;
      *q++ = *p++;
    }
  }

  // put checksum into packet
  *q++ = pkt->chk_sum;

  // Send packet (do not wait for reply)
  SendPacket( pkt );

  return( SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Take packet and start transmission                                   */
/*---------------------------------------------------------------------------*/

int
SendPacket( packet_t *pkt ) {
  // Transmit
  RobotC_WriteBuf( MyComms.port, pkt->data, pkt->msg_len + 2 );

  return( SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Check for serial port for some data                                  */
/*---------------------------------------------------------------------------*/

int
ReceiveData() {
  int         data;
  bool        delim_found = false;
  int         i;

  // If we don't have recv any data, then just return
  if( RobotC_PeekInput() == RX_NO_DATA ) {
    return( RX_NO_DATA );
  }

  // We've recv'd something, so reset pointer first
  // initial partial packets will drop
  MyComms.rxcnt = 0;

  // Read everything available
  do {
    data = RobotC_GetChar();
    if( data >= 0 ) {
      if (data == '[') {
        MyComms.rxcnt = 0;
      }
      MyComms.rxbuf[MyComms.rxcnt++] = data;
      if( MyComms.rxcnt == RX_BUF_SIZE )
        return( RX_BUF_ERR );
      if (data == ']') {
        delim_found = true;
        for (i = 0; i < MyComms.rxcnt; i++) {
          _extra_buf[i] = MyComms.rxbuf[i];
        }
        _extra_buf[i] = 0;
        _extra_buf_len = MyComms.rxcnt;
        MyComms.rxcnt = 0;
        break;
      }
    }
  } while( data >= 0 );

  if (delim_found && _extra_buf[0] == '[') {
    return( _extra_buf_len );
  } else {
    return 0;
  }
 }

/*---------------------------------------------------------------------------*/
/*      Select message based on cmd                                          */
/*---------------------------------------------------------------------------*/

message_t *
GetMessageStructure(unsigned char cmd) {
  switch (cmd) {
    case CMD_CONTROL_MOTOR_VALUES:
      return &_motor_value_message;

    default:
      return NULL;
  }
}

/*---------------------------------------------------------------------------*/
/*      Received some data so start to decode packet (format2)               */
/*---------------------------------------------------------------------------*/

int
ReceiveFmt2Packet() {
  int             i;
  message_t       *message;
  char            m1, m2, *p;
  unsigned char   chk_sum;

  message = &_motor_value_message;
  message->length = _extra_buf_len;

  if (_extra_buf[0] != '[' || _extra_buf[_extra_buf_len-1] != ']') {
    return( FAILURE );
  }

  if (_extra_buf[1] != CMD_CONTROL_MOTOR_VALUES || (_extra_buf_len != 20+5)) {
    return( FAILURE );
  }

  if (!ISHEX4(_extra_buf[22]) || !ISHEX4(_extra_buf[23])) {
    return FAILURE;
  }
  chk_sum = ((FROMHEX4(_extra_buf[22]) << 4) + FROMHEX4(_extra_buf[23])) ^ '\n';

  _extra_buf[22] = 0;
  _extra_buf[23] = 0;
  for (i = 0; i < 25; i++) {
    chk_sum ^= _extra_buf[i];
  }
  if (chk_sum != 0) {
    return FAILURE;
  }

  p = &_extra_buf[2];
  for (i = 0; i < 10; i++) {
    m1 = *p++;
    m2 = *p++;

    if (!ISHEX4(m1) || !ISHEX4(m2)) {
      return FAILURE;
    }
    message->data[i] = (char)((FROMHEX4(m1) << 4) + FROMHEX4(m2));
  }

  DecodeMessage(message);

  return( SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Received some data so start to decode packet                         */
/*---------------------------------------------------------------------------*/

void
ReceivePacket() {
  int             i;
  packet_t        *RxPak;
  message_t       *message;
  char            ch;

  RxPak = &MyComms.RxPak;

  for (i = 0; i < MyComms.rxcnt; i++) {
    ch = MyComms.rxbuf[i];
    switch( RxPak->msg_cnt ) {
      case    0: // should be preamble 1
        if( ch == PREAMBLE1 ) {
          RxPak->msg_cnt = 1;
        } else {
          RxPak->msg_cnt = 0;
        }
        break;

      case    1: // should be preamble 2
        if( ch == PREAMBLE2 ) {
          RxPak->chk_sum = PREAMBLE1 ^ PREAMBLE2;
          RxPak->msg_cnt = 2;
        } else {
          RxPak->msg_cnt = 0;
        }
        break;

      case    2: // cmd
        message = GetMessageStructure(ch);
        if ( message != NULL ) {
          RxPak->chk_sum = RxPak->chk_sum ^ ch;
          RxPak->msg_cnt++;
        } else {
          RxPak->msg_cnt = 0;
        }
        break;

      case    3: // len
        message->length = ch;
        RxPak->msg_len = ch + 5;
        RxPak->chk_sum = RxPak->chk_sum ^ ch;
        RxPak->msg_cnt++;
        break;

      default:
        if( RxPak->msg_cnt < (RxPak->msg_len) ) {
          message->data[RxPak->msg_cnt-4] = ch;
          RxPak->chk_sum = RxPak->chk_sum ^ ch; // last byte will be the checksum
          RxPak->msg_cnt++;
        }
        break;
    }

    // Look for packet end and run cmd
    if( (RxPak->msg_cnt > 0) && (RxPak->msg_cnt == RxPak->msg_len) ) {
      if( RxPak->chk_sum == 0 ) {
        DecodeMessage( message );
      }
      RxPak->msg_cnt = 0;
    }
  }
}

/*---------------------------------------------------------------------------*/
/*      Decode a received packet and take appropriate action                 */
/*---------------------------------------------------------------------------*/

void
DecodeMessage( message_t *message ) {
  int         i;
  if ( message == NULL ) {
    return;
  }

  // Read data
  switch (message->cmd) {
    case CMD_CONTROL_MOTOR_VALUES:
      for (i = 0; i < 10; i++) {
        _motor_values[i] = (int)(message->data[i]) - 0x7F;
      }
      RunMotors(_motor_values);
      break;

    default:
      return;
  }
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
    ReceiveFmt2Packet();
  }

  // Send out a packet no matter what (buffer overflow problems?)
  if (MyComms.txto == 0) {
    SendFmt2SensorMessage();
    MyComms.txto = 5; // reset the tx timeout to 10mS
  } else {
    MyComms.txto--;
  }

  return( SUCCESS );
}

#endif
