#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <dirent.h>
#include <stdio.h>
#include <time.h>

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

#define SER_PORT                 "/dev/ttyTHS1"
#define BAUD                     B115200

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
    char            port[128];    // which Uart
    int             fd;           // fd
    long            baud;         // baud rate of channel
    int             parity;       // 0, odd, even
    int             tcount;       // number of timeouts

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

    pthread_t       comm_task;
    int             task_running;

} comms_t;


// Prototypes
int         SerialInit( char *port=SER_PORT, long baud=BAUD );
int         _serial_setattr(int parity, long baud);
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
int         _serial_update();

void      ReadSensors();
void      RunMotors(int *mtrV);

#define SerialWriteSensor(name) UARTWriteSensorValue(SensorType(name), SensorValue(name))

/*---------------------------------------------------------------------------*/
/*  END HEADER                                                               */
/*---------------------------------------------------------------------------*/

static comms_t MyComms;

// messages
static message_t  _debug_message100;
static message_t  _motor_value_message;
static message_t  _sensor_value_message;

mutex_t           sensor_lock;
mutex_t           motor_lock;

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
/*  Init                                                                     */
/*---------------------------------------------------------------------------*/

int SerialInit( char *port, long baud ) {
  int i;

  // Note which serial port we are using
  strcpy(MyComms.port, port);
  MyComms.fd = open(MyComms.port, O_RDWR | O_NONBLOCK);
  if (MyComms.fd == -1) {
    return FAILURE;
  }

  // zero motor and sensor values
  memset(_motor_values, 0, sizeof(_motor_values));
  memset(_sensor_values, 0, sizeof(_sensor_values));

  // Init the serial port here
  if (_serial_setattr(0, baud) == FAILURE) {
    close(MyComms.fd);
    MyComms.fd = -1;
    return FAILURE;
  }

  // Get rid of garbage (time: 900msec)
  usleep(900000);
  tcflush(connection->fd, TCIOFLUSH);

  // start comm task
  MyComms.task_running = true;
  if (pthread_mutex_init(&motor_lock, 0) != SUCCESS) {
    return FAILURE;
  }
  if (pthread_mutex_init(&sensor_lock, 0) != SUCCESS) {
    return FAILURE;
  }
  return pthread_create(&MyComms.comm_task, 0, &_serial_update, NULL);
}

void SerialDeInit() {
  if (MyComms.task_running) {
    MyComms.task_running = 0;
    usleep(500000);
    pthread_join(MyComms.comm_task, NULL);

    pthread_mutex_destroy(&motor_lock);
    pthread_mutex_destroy(&sensor_lock);
  }
}

/*---------------------------------------------------------------------------*/
/*      Initialize serial port                                               */
/*---------------------------------------------------------------------------*/

int _serial_setattr(int parity, long _baud) {
  struct termios tty;

  if (tcgetattr(MyComms.fd, &tty) == -1) {
    return FAILURE;
  }

  speed_t baud = B9600;
  switch (_baud) {
    case 4800:    baud = B4800;   break;
    case 9600:    baud = B9600;   break;
#ifdef B14400
    case 14400:   baud = B14400;  break;
#endif
    case 19200:   baud = B19200;  break;
#ifdef B28800
    case 28800:   baud = B28800;  break;
#endif
    case 38400:   baud = B38400;  break;
    case 57600:   baud = B57600;  break;
    case 115200:  baud = B115200; break;
    default:      baud = _baud;   break;
  }
  cfsetospeed(&tty, baud);
  cfsetispeed(&tty, baud);

  // set new attributes
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(MyComms.fd, TCSANOW, &tty) == -1) {
    return FAILURE;
  }
  if (tcsetattr(MyComms.fd, TCSAFLUSH, &tty) == -1) {
    return FAILURE;
  }

  MyComms.parity = parity;
  MyComms.baud = baud;
  return SUCCESS;
}

/*---------------------------------------------------------------------------*/
/*      Get motor values and send them                                       */
/*---------------------------------------------------------------------------*/

int SendFmt2MotorMessage() {
  int             i;
  char            *buf = &_motor_value_message.data[0], *p;
  unsigned char   chk_sum = 0, v, ch;

  // We are going to have to use a special fmt to compose the message
  buf[0] = 0;  // in case of de-sync
  buf[1] = 0;  // in case of de-sync
  buf[2] = '[';
  buf[3] = CMD_CONTROL_MOTOR_VALUES;

  mutex_lock(&motor_lock, NULL);
  p = &buf[4];
  for (i = 0; i < 10; i++) {
    v = _motor_values[i] + 0x7F;

    // write to buf
    ch = (v >> 4) & 0xF;
    *p++ = TOHEX4(ch);

    ch = v & 0xF;
    *p++ = TOHEX4(ch);
  }
  mutex_unlock(&motor_lock, NULL);

  buf[24] = 0;
  buf[25] = 0;
  buf[26] = ']';
  buf[27] = '\n';

  for (i = 2; i < 28; i++) {
    chk_sum ^= buf[i];
  }
  v = chk_sum & 0xF;
  buf[25] = TOHEX4(v);
  v = (chk_sum >> 4) & 0xF;
  buf[24] = TOHEX4(v);

  write(MyComms.fd, buf, 28);

  return( SUCCESS );
}

/*---------------------------------------------------------------------------*/
/*      Check for serial port for some data                                  */
/*---------------------------------------------------------------------------*/

int ReceiveData() {
  int         data;
  int         delim_found = 0;
  int         i;
  char        *rxbuf = MyComms.rxbuf;
  char        *storebuf = MyComms.RxPak.data;
  int         _bufmax = MSGLEN * 2;
  int         bytesRead;
  int         bytesStored;
  int         analyzeBuffer;

  /* update buffer constantly (be careful of overflow!) */
  analyzeBuffer = 0;
  while ((bytesRead = read(MyComms.fd, rxbuf, MSGLEN)) > 0) {
    analyzeBuffer = 1;
    rxbuf[bytesRead] = '\0';
    bytesStored = strlen(storebuf); /* no \0 */

    while (bytesStored + bytesRead >= (_bufmax)) {
      /* shorten it by only half of the readmax value */
      bytesStored -= MSGLEN / 2;
      memmove(storebuf, &storebuf[MSGLEN / 2], bytesStored * sizeof(char));
      storebuf[bytesStored] = '\0';
    }

    strcat(storebuf, rxbuf);
  }

  if (analyzeBuffer) {
    char *end_index = strchr(storebuf, '\n');
    if (end_index != NULL && start_index != NULL) {
      char *start_index = strrchr(end_index, '[');
      end_index[0] = '\0';
      end_index = &end_index[1];
      if (start_index != NULL) {
        strcpy(_extra_buf, start_index);
        _extra_buf_len = (int)(end_index - start_index);
        delim_found = 1;
      }
      memmove(storebuf, end_index, (strlen(end_index) + 1) * sizeof(char));
    }
  }

  if (delim_found) {
    return _extra_buf_len;
  } else {
    return 0;
  }
 }

/*---------------------------------------------------------------------------*/
/*      Received some data so start to decode packet (format2)               */
/*---------------------------------------------------------------------------*/

int ReceiveFmt2Packet() {
  int             i, v;
  message_t       *message;
  char            ch;
  unsigned char   chk_sum = 0;
  int             length;
  int             num_bytes;

  short svalues[20];
  int scnt = 0;

  message = &_sensor_value_message;
  message->length = _extra_buf_len;

  if (_extra_buf[0] != '[' || _extra_buf[_extra_buf_len-1] != ']') {
    return( FAILURE );
  }

  if (_extra_buf[1] != CMD_STATUS_SENSOR_VALUES || _extra_buf_len < 7) {
    return( FAILURE );
  }

  if (!ISHEX4(_extra_buf[2]) || !ISHEX4(_extra_buf[3])) {
    return FAILURE;
  }
  length = (FROMHEX4(_extra_buf[2]) << 4) + FROMHEX4(_extra_buf[3]);
  if (_extra_buf_len - 1 != length) {
    return FAILURE;
  }

  if (!ISHEX4(_extra_buf[_extra_buf_len-3]) ||
      !ISHEX4(_extra_buf[_extra_buf_len-2])) {
    return FAILURE;
  }
  chk_sum = (FROMHEX4(_extra_buf[_extra_buf_len-3]) << 4) +
            (FROMHEX4(_extra_buf[_extra_buf_len-2])) ^ '\n';

  _extra_buf[_extra_buf_len-3] = 0;
  _extra_buf[_extra_buf_len-2] = 0;

  for (i = 0; i < _extra_buf_len; i++) {
    chk_sum ^= _extra_buf[i];
  }
  if (chk_sum != 0) {
    return FAILURE;
  }

  // all basic checks are now out of the way
  for (i = 4; i < _extra_buf_len - 5;) {
    ch = _extra_buf[i++];
    switch (ch) { // sensor decode
      case 'd':
        num_bytes = 1;
        break;
      case 'a':
      case 's':
        num_bytes = 3;
        break;
      case 'q':
        num_bytes = 4;
        break;
      case 'u':
        num_bytes = 2;
        break;
      default:
        return FAILURE;
    }
    v = 0;
    while (num_bytes > 0) {
      ch = _extra_buf[i++];
      if (!ISHEX4(ch)) {
        return FAILURE;
      }
      v = (v << 4) + (int)FROMHEX4(ch);
    }
    svalues[scnt++] = v;
  }

  mutex_lock(&sensor_lock, NULL);
  memcpy(_sensor_values, svals, sizeof(&_sensor_values));
  _sensor_cnt = scnt;
  mutex_unlock(&sensor_lock, NULL);

  return SUCCESS;
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

int _serial_update() {
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
