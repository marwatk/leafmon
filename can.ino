#define SOFT_SERIAL

#include <SdFat.h>
#ifdef SOFT_SERIAL
#include <AltSoftSerial.h>
#endif
#include <mcp_can.h>
#include <RTClib.h>
#include <digitalWriteFast.h>

/*
 * File size 5 MB
Buffer size 512 bytes
Starting write test, please wait.

write speed and latency
speed,max,min,avg
KB/Sec,usec,usec,usec
177.33,94440,2416,2873
189.83,37608,2384,2683

Starting read test, please wait.

read speed and latency
speed,max,min,avg
KB/Sec,usec,usec,usec
278.46,3808,1800,1824
278.49,3632,1800,1824

 */

//Bluetooth
/*
 * VCC - 3V
 * RXD - 9
 * TXD - 8
 * 
 * 
 */
/* CAN0
 *  
 *  VCC - 3.3V
 *  CS - 2
 *  SO - 12
 *  SI - 11
 *  SCK - 13
 *  INT - 3
 *  
 */

/* CAN1
 *  VCC - 3.3V
 *  CS - 4
 *  SO - 12
 *  SI - 11
 *  SCK - 13
 *  INT - 5
 */

/* CAN3
 *  VCC - 3.3V
 *  CS - 6
 *  SO - 12
 *  SI - 11
 *  SCK - 13
 *  INT - 7
 */

 /* BT

 *
 * 
 */
 
 /* RTC

SCL: A5
SDA: A4
 */

/*
 * SD Card
 * CS - 10
 * DO - 12
 * DI - 11
 * CLK - 13
 */

 /* Leaf
Blue
 CAR H 6
CAR L 14

Green
EV H 13
EV L 12

Brown
AV H 11
AV L 3

POWER + 16
SWITCH + 8

Chasis 4
Ground 5
*/
//25ma at 3.3V


//https://github.com/marwatk/CAN_BUS_Shield otherwise we're using https://github.com/marwatk/MCP_CAN_lib
#define CAN_BUS_SHIELD 1

#define SIZE_IRRELEVANT_8 0xff

#define CAN_MSG_LOG_HEADER PSTR("C#=")
#define DATE_LOG_HEADER PSTR("T#=")
#define NEWLINE PSTR("\n")
//#define BUFFER_SD_WRITES
//#define LOG_CANS 1

RTC_DS3231 rtc;

MCP_CAN CAR_CAN(2);
MCP_CAN EV_CAN(4);
MCP_CAN AV_CAN(6);

#define CAR_INT 3
#define EV_INT 5
#define AV_INT 7

#ifdef CAN_BUS_SHIELD
//#define MY_CAN_MODE MODE_LISTENONLY
#define MY_CAN_MODE MODE_NORMAL
#else
#define MY_CAN_MODE MCP_LISTENONLY
//#define MY_CAN_MODE MCP_NORMAL
#endif //CAN_BUS_SHIELD

boolean carEnabled = false;
boolean evEnabled = false;
boolean avEnabled = false;

#define SD_CS 10

//CAN Message buffer and content
unsigned char read;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
bool isRemoteRequest = false;
unsigned char isExtended = false;

//Stats to monitor
unsigned char odo1;
unsigned char odo2;
unsigned char odo3;
unsigned char SOH;
unsigned char ActiveFuelBars;
unsigned char batteryVolts12V = 0;
unsigned char SOC;
unsigned char chargeState = 0;
#define CHARGING_BIT 1
#define PLUGGED_IN_BIT 2

//Read/command buffer
#define BUF_SIZE 16
char buf[BUF_SIZE];

//Message buffer
char msgString[128];

//Time of loop start
unsigned long loopStart = 0;

//Logging settings
#define MAX_FILE_SIZE 10485760 //10MB
//#define MAX_FILE_SIZE 1048576 //1MB
//#define MAX_FILE_SIZE 10240 //10k
boolean loggingEnabled = false;
unsigned int fileNumber = 0;
unsigned long fileBytesWritten = 0;
boolean fileOpen = false;

#ifdef SOFT_SERIAL
AltSoftSerial bt;
#endif
SdFat sd;
SdFile file;

#define TEST_INTERVAL_MICROS 600

unsigned long lastMessages[] = { 0, 0, 0 };
unsigned long missed[] = { 0, 0, 0 };
unsigned long lastReport = 0;
unsigned long msgsReceived = 0;
unsigned long loopCount = 0;
unsigned long idleLoops = 0;
unsigned long lastWriteTime = 0;
#define REPORT_INTERVAL 3000

boolean serialLog = false;
boolean textLog = false;

void dateTime(uint16_t* date, uint16_t* time) {
  DateTime now = rtc.now();
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());
  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void setup() {
  Serial.begin(500000);
  Serial.print( F("Setting up\n") );
  Serial.print( F("Starting RTC\n") );
  if( !rtc.begin() ) {
    Serial.println(F("Couldn't find rtc"));
    while( 1 );
  }
  else {
    if (rtc.lostPower()) {
      Serial.println(F("RTC lost power, lets set the time!"));
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }
  Serial.print( F("Starting SD Card\n") );
  if (sd.begin(SD_CS, SD_SCK_MHZ(50))) {
    log(F("SD Card Configured\n"));
  }
  else {
    log(F("Error configuring SD Card\n"));
  }

  delay( 2000 ); //Wait for BT to initialize
  #ifdef SOFT_SERIAL
  setupBt();
  #endif
  resetCans();
}

void loop() {
  size_t timeLen;
  loopStart = millis();
  boolean idleLoop = true;
  
  while( !digitalReadFast( CAR_INT ) && carEnabled ) {
    readCanMessage( CAR_CAN, 'C', 0 );
    msgsReceived++;
    idleLoop = false;
  }
  while( !digitalReadFast( EV_INT ) && evEnabled ) {
    readCanMessage( EV_CAN, 'E', 1 );
    msgsReceived++;
    idleLoop = false;
  }
  while( !digitalReadFast( AV_INT ) && avEnabled ) {
    readCanMessage( AV_CAN, 'A', 2 );
    msgsReceived++;
    idleLoop = false;
  }
  
 #ifdef SOFT_SERIAL
  while( (read = bufferedRead( buf, BUF_SIZE, bt )) > 0 ) {
    log( F("From Bluetooth: [" ) );
    log( buf );
    log( F("]\n" ) );
    handleInput( buf );
    idleLoop = false;
  }
  #endif
  while( (read = bufferedSerialRead( buf, BUF_SIZE )) > 0 ) {
    log( F("From Serial Port: [" ) );
    log( buf );
    log( F("]\n" ) );
    handleInput( buf );
    idleLoop = false;
  }
  if( idleLoop ) {
    idleLoops++;
  }
  loopCount++;
  if( idleLoop && millis() - lastReport > REPORT_INTERVAL ) {
    //sprintf_P( msgString, PSTR("Msgs: %lu, loops: %lu  idle: %lu, lwt: %lu, lrt: %lu, miss: %lu\n" ), msgsReceived, loopCount, idleLoops, lastWriteTime, lastReadTime, missed[0] + missed[1] + missed[2] );
    sprintf_P( msgString, PSTR("Msgs: %lu, loops: %lu  idle: %lu, lwt: %lu\n" ), msgsReceived, loopCount, idleLoops, lastWriteTime );
    log( msgString );
    #ifdef SOFT_SERIAL
      bt.print( msgString );
    #endif
    lastReport = millis();
    idleLoops = 0;
    loopCount = 0;
    msgsReceived = 0;
    missed[0] = 0;
    missed[1] = 0;
    missed[2] = 0;
    if( serialLog ) {
      timeMapToSerial();
    }
  }
}
int sentMessageNumber = 0;
int recvMessageNumber = 0;

void switchFile() {
  if( fileOpen ) {
    file.close();
    fileOpen = false;
    fileBytesWritten = 0;
  }
  if( loggingEnabled ) {
    do {
      fileNumber++;
      sprintf_P( msgString, PSTR("%08d.log"), fileNumber );
    } while( sd.exists( msgString ) );
    log( F("Logging to file ") );
    log( msgString );
    log( F("\n") );
    SdFile::dateTimeCallback(dateTime);
    if( file.open( msgString, O_CREAT | O_WRITE | O_EXCL ) ) {
      fileBytesWritten = 0;
      fileBytesWritten = timeMapToMsgString();
      file.write( (void*)msgString, fileBytesWritten ); 
      fileOpen = true;
    }
    else {
      log( F("Error opening logging file, disabling logging" ) );
      loggingEnabled = false;
      fileOpen = false;
    }
  }
}

size_t writeLongToFile( unsigned long val ) {
  file.write( (void*)&val, sizeof( unsigned long ) );
  return sizeof( unsigned long );
}

void longToChar( char * target, unsigned long val ) {
  memcpy( target, &val, sizeof( unsigned long ) );
  target[sizeof(unsigned long)] = 0;
}

void resetCans() {
  #ifdef CAN_BUS_SHIELD
  //No one shot mode here...
  if( CAR_CAN.begin(CAN_500KBPS, MCP_8MHz, MY_CAN_MODE) == CAN_OK ) {
    carEnabled = true;
  }
  if( EV_CAN.begin(CAN_500KBPS, MCP_8MHz, MY_CAN_MODE) == CAN_OK ) {
    evEnabled = true;
    }
  if( AV_CAN.begin(CAN_500KBPS, MCP_8MHz, MY_CAN_MODE) == CAN_OK ) {
    avEnabled = true;
  }
  #else
  if( CAR_CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ) {
    CAR_CAN.setMode(MY_CAN_MODE);
    CAR_CAN.enOneShotTX();
    carEnabled = true;
  }
  if( EV_CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ) {
    EV_CAN.setMode(MY_CAN_MODE);
    EV_CAN.enOneShotTX();
    evEnabled = true;
    }
  if( AV_CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ) {
    AV_CAN.setMode(MY_CAN_MODE);
    AV_CAN.enOneShotTX();
    avEnabled = true;
  }
  #endif

  if( carEnabled ) {
    log( F("CAR can enabled\n") );
  }
  else {
    log( F("CAR can failed\n" ) );
  }
  if( evEnabled ) {
    log( F("EV can enabled\n") );
  }
  else {
    log( F("EV can failed\n" ) );
  }

  if( avEnabled ) {
    log( F("AV can enabled\n") );
  }
  else {
    log( F("AV can failed\n" ) );
  }


}

/* This is using an SH-HC-08 (which works on 3.3V)
 *  https://smile.amazon.com/gp/product/B01N4P7T0H
 *  http://www.dsdtech-global.com/2017/08/sh-hc-08.html
 *  https://drive.google.com/file/d/0B4urklB65vaCcEVyMm5haVVpMUk/view?usp=sharing
 *  Changes will likely be needed with other version of BLE modules
 */


void canMsgToTextSerial( char canLabel, unsigned char canCounter ) {
  int i;
//  sprintf_P(msgString, PSTR("%lu: (%d): Extended ID: 0x%.8lX  DLC: %1d  Data:"), (unsigned long)loopStart, canCounter, rxId, len);
  sprintf_P(msgString, PSTR("%lu: (%d): Standard ID: 0x%.3lX       DLC: %1d  Data:"), (unsigned long)loopStart, canCounter, rxId, len);
  Serial.write(msgString);
  if( isRemoteRequest ) {    
    sprintf_P(msgString, PSTR(" REMOTE REQUEST FRAME"));
    Serial.write(msgString);
  } else {
    for(i = 0; i<len; i++){
      sprintf_P(msgString, PSTR(" 0x%.2X"), rxBuf[i]);
      Serial.write(msgString);
    }
  }
  Serial.print( F("\n") );
  
}

void readCanMessage( MCP_CAN &can, char canLabel, unsigned char canCounter ) {
  int i = 0;
  #ifdef CAN_BUS_SHIELD
    can.readMsgBufID( &rxId, &len, rxBuf ); //72-76 micros 
    isRemoteRequest = can.isRemoteRequest();
    isExtended = can.isExtendedFrame();
  #else
    can.readMsgBuf( &rxId, &isExtended, &len, rxBuf ); //152 micros
    isRemoteRequest = can.isRequestMessage();
  #endif

  /*
  unsigned char curMessage = rxBuf[0];
  if( curMessage != 0 && lastMessages[canCounter] + 1 != curMessage ) {
    missed[canCounter] += curMessage - lastMessages[canCounter] - 1;
  }
  lastMessages[canCounter] = curMessage;
  */
  
  //Process Message
  /*
  switch( rxId ) {
    case 0x292:
      batteryVolts12V = rxBuf[3];
      break;
    case 0x5b3:
      SOH = rxBuf[1];
      ActiveFuelBars = rxBuf[6];
      break;
    case 0x5c5:
      odo1 = rxBuf[1];
      odo2 = rxBuf[2];
      odo3 = rxBuf[3];
      break;
  }
  */
  
  if( loggingEnabled && fileOpen ) {
    int written = 0;
    unsigned long start = micros();
    size_t msgLen = canMsgToMsgString( canLabel, canCounter );
    written = file.write( (void*)msgString, msgLen );
    lastWriteTime = micros() - start;
    if( serialLog ) {
      while( i < msgLen ) {
        Serial.write( msgString[i++] );
      }
    }
 
    if( written == -1 ) {
      log( F("Write error. Closing file") );
      loggingEnabled = false;
      switchFile();
    }
    else {
      fileBytesWritten += written;
      if( fileBytesWritten > MAX_FILE_SIZE ) {
        switchFile();
      }
    }
  }
  else if( serialLog ) {
    size_t msgLen = canMsgToMsgString( canLabel, canCounter );
    i = 0;
    while( i < msgLen ) {
      Serial.write( msgString[i++] );
    }
  }
  if( textLog ) {
    canMsgToTextSerial( canLabel, canCounter );
  }

}

size_t canMsgToMsgString( char canLabel, unsigned char canCounter ) {
  size_t pos = 0;
  pos += mconcatStr( msgString, pos, CAN_MSG_LOG_HEADER, SIZE_IRRELEVANT_8 );
  pos += mconcatLong( msgString, pos, loopStart );
  pos += mconcatChar( msgString, pos, (unsigned char)canCounter );
  pos += mconcatLong( msgString, pos, (unsigned long)rxId );
  pos += mconcatChar( msgString, pos, (unsigned char)len );
  if( len > 0 ) {
    pos += mconcatMem( msgString, pos, rxBuf, len );
  }
  pos += mconcatStr( msgString, pos, NEWLINE, SIZE_IRRELEVANT_8 );
  return pos;
}


size_t timeMapToMsgString() {
  size_t pos = 0;
  pos += mconcatStr( msgString, pos, DATE_LOG_HEADER, SIZE_IRRELEVANT_8 );
  pos += mconcatLong( msgString, pos, millis() );
  pos += mconcatLong( msgString, pos, rtc.now().unixtime() );
  pos += mconcatStr( msgString, pos, NEWLINE, SIZE_IRRELEVANT_8 );
  return pos;
}

void timeMapToSerial() {
  size_t i = 0;
  size_t len = timeMapToMsgString();
  while( i < len ) {
    Serial.write( msgString[i++] );
  }
}

size_t mconcatMem( void* dest, size_t outPos, const void* src, uint8_t count ) {
  dest += outPos;
  memcpy( dest, src, count );
  return count;
}

size_t mconcatChar( char* dest, size_t outPos, unsigned char value ) {
  dest += outPos;
  memcpy( dest, &value, sizeof( unsigned char ) );
  return sizeof( unsigned char );
}  

size_t mconcatLong( char* dest, size_t outPos, unsigned long value ) {
  dest += outPos;
  memcpy( dest, &value, sizeof( unsigned long ) );
  return sizeof( unsigned long );
}

size_t mconcatStr( char* dest, size_t outPos, char* src, uint8_t size ) {
  bool size_known = (size != SIZE_IRRELEVANT_8);
  size_t read = 0;
  dest += outPos;
  char ch = '.';
  size_t written = 0;
  while (written < size && ch != '\0')
  {
      ch = src[read++];
      if( ch != '\0' ) {
        *dest++ = ch;
        written++;
      }
  }
  if (size_known)
  {
      while (written < size)
      {
          *dest++ = 0;
          written++;
      }
  }

  return written;
}  

size_t mconcatStr( char* dest, size_t outPos, PGM_P src, uint8_t size ) {
  bool size_known = (size != SIZE_IRRELEVANT_8);
  const char* read = src;
  char* write = dest;
  write += outPos;
  char ch = '.';
  size_t written = 0;
  while (written < size && ch != '\0')
  {
      ch = pgm_read_byte(read++);
      if( ch != '\0' ) {
        *write++ = ch;
        written++;
      }
  }
  if (size_known)
  {
      while (written < size)
      {
          *write++ = 0;
          written++;
      }
  }

  return written;

}


#ifdef SOFT_SERIAL
void setupBt() {
  bt.begin(9600); //Defaults to 9600
  sendBtAtCommand( F("AT+NAMELeafLogger") );
  readBtCommandResponse();
  sendBtAtCommand( F("AT+BAUD6") ); //4=9600, 5=19200, 6=38400, 7=57600, 8=115200
  readBtCommandResponse();
  bt.begin(38400);
  sendBtAtCommand( F("AT") );
  readBtCommandResponse();
}

void sendBtAtCommand( const __FlashStringHelper* command ) {
  Serial.print( F("BT Command: ") );
  Serial.print( command );
  Serial.print( F("\n") );
  bt.print( command );
  bt.print( F("\n") );
}
void readBtCommandResponse() {
  delay( 100 );
  while( (read = bufferedRead( buf, BUF_SIZE, bt )) > 0 ) {
    log( F("BT  Output: ") );
    log( buf );
    log( F("\n") );
    delay( 10 );
  }
}

int bufferedRead( char* buf, int maxLen, AltSoftSerial &ser ) {
  maxLen--;
  int len = 0;
  int avail = 0;
  
  while( (avail = ser.available()) > 0 && len < maxLen) {
    while( avail-- > 0 ) {
      buf[len++] = (char)ser.read();
    }
    delayMicroseconds( 400 );
  }
  buf[len] = 0;
  return len;
}

#endif //SOFT_SERIAL

int bufferedSerialRead( char* buf, int maxLen ) {
  maxLen--;
  int len = 0;
  int avail = 0;
  
  while( (avail = Serial.available()) > 0 && len < maxLen) {
    while( avail-- > 0 ) {
      buf[len++] = (char)Serial.read();
    }
    delayMicroseconds( 200 );
  }
  buf[len] = 0;
  return len;
}

void handleInput( char* input ) {
  if( !strncmp_P( input, PSTR("reset"), 5 ) ) {
    log( F("Reset command from bluetooth\n" ) );
  }
  else if( !strncmp_P( input, PSTR("stop"), 4 ) ) {
    log( F("Stop command from bluetooth\n" ) );
    loggingEnabled = false;
    switchFile();
    log( F("Logging ended") );
  }
  else if( !strncmp_P( input, PSTR("start"), 5 ) ) {
    log( F("Start command from bluetooth\n" ) );
    loggingEnabled = true;
    switchFile();
    sprintf_P( msgString, PSTR( "Log enabled, %lu bytes written, filename %08d.log\n"), (unsigned long)fileBytesWritten, fileNumber );
    log( msgString );
  }
  else if( !strncmp_P( input, PSTR("status"), 5 ) ) {
    log( F("Status command from bluetooth\n" ) );
    if( loggingEnabled ) {
      sprintf_P( msgString, PSTR( "Log enabled, %lu bytes written, filename %08d.log\n"), (unsigned long)fileBytesWritten, fileNumber );
      log( msgString );
    }
    else {
      log( F("Logging disabled") );
    }
  }
  else if( !strncmp_P( input, PSTR("serial"), 6 ) ) {
    serialLog = !serialLog;
  }
  else if( !strncmp_P( input, PSTR("text"), 4 ) ) {
    textLog = !textLog;
  }
  else if( !strncmp_P( input, PSTR("send"), 4 ) ) {
    sprintf_P( msgString, PSTR( "AMessage" ) );
    CAR_CAN.sendMsgBuf( 0x3d3, 0, 8, msgString, false );
  }
  else {
    log( F("Received unknown bluetooth command" ) );
  }
}

void log( const __FlashStringHelper* msg) {
  Serial.print( msg );
  if( fileOpen ) {
    file.print( msg );
  }
}

void log( char* b ) {
  Serial.print( b );
  if( fileOpen ) {
    file.print( b );
  }
}
