#include <SdFat.h>
#include <AltSoftSerial.h>
#include <mcp_can.h>
#include <RTClib.h>

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
//25ma at 3.3V

#define SIZE_IRRELEVANT 0x7fffffff

#define CAN_MSG_LOG_HEADER PSTR("C#=")
#define DATE_LOG_HEADER PSTR("T#=")
#define NEWLINE PSTR("\n")
//#define LOG_CANS 1

RTC_DS3231 rtc;

MCP_CAN cans[3] = { //CS pins
  MCP_CAN(2),
  MCP_CAN(4),
  MCP_CAN(6),
};

#define SD_CS 10

char canInterupts[] = {3, 5, 7};
bool canEnabled[] = {false,false,false};
unsigned char canCounter = 0;

#define SOAK_TEST 1
#define NUM_CANS 3
#define BT_POWER_PIN A3
//#define CAN0_INT 7 //CAN INT
//MCP_CAN CAN0(10); //CAN CS

//CAN Message buffer and content
unsigned char read;
long unsigned int rxId;
long unsigned int realRxId;
unsigned char len = 0;
unsigned char rxBuf[8];
bool isRemoteRequest = false;

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

//Read buffer
#define BUF_SIZE 16
char buf[BUF_SIZE];

//Message buffer
char msgString[64];

//Logging settings
#define MAX_FILE_SIZE 10485760 //10MB
//#define MAX_FILE_SIZE 10240 //10k
boolean loggingEnabled = false;
unsigned int fileNumber = 0;
size_t fileBytesWritten = 0;
boolean fileOpen = false;

AltSoftSerial bt;
SdFat sd;
SdFile file;

#define TEST_INTERVAL_MILLIS 1
boolean runTest = false;
unsigned long lastSent = 0;

unsigned long lastReport = 0;
unsigned long msgsReceived = 0;
unsigned long loopCount = 0;
unsigned long idleLoops = 0;
#define REPORT_INTERVAL 5000


void setup() {
  Serial.begin(9600);
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

  delay( 200 ); //Wait for BT to initialize
  setupBt();

  resetCans();
}

void loop() {
  boolean idleLoop = true;
  if( fileBytesWritten > MAX_FILE_SIZE ) {
    switchFile();
    idleLoop = false;
  }
  
  for( canCounter = 0; canCounter < NUM_CANS; canCounter++ ) {
    if( !digitalRead(canInterupts[canCounter]) && canEnabled[canCounter] ) {
      readCanMessage( cans[canCounter] );
      msgsReceived++;
      idleLoop = false;
    }
  }
  while( (read = bufferedRead( buf, BUF_SIZE, bt )) > 0 ) {
    log( F("From Bluetooth: [" ) );
    log( buf );
    log( F("]\n" ) );
    handleBtInput( buf );
    idleLoop = false;
  }
  while( (read = bufferedSerialRead( buf, BUF_SIZE )) > 0 ) {
    log( F("From Serial Port: [\n" ) );
    log( buf );
    log( F("]\n" ) );
    handleBtInput( buf );
    idleLoop = false;
  }
  if( runTest && idleLoop) {
    if( millis() - lastSent > TEST_INTERVAL_MILLIS ) {
      sendTest( 0 );
      idleLoop = false;
      lastSent = millis();
    }
  }
  if( idleLoop ) {
    idleLoops++;
  }
  loopCount++;
  if( millis() - lastReport > REPORT_INTERVAL ) {
    sprintf_P( msgString, PSTR("Messages received: %lu, loopCount: %lu, idleLoops: %lu\n"), msgsReceived, loopCount, idleLoops );
    log( msgString );
    lastReport = millis();
    idleLoops = 0;
    loopCount = 0;
    msgsReceived = 0;
  }
}
int sentMessageNumber = 0;
int recvMessageNumber = 0;
void sendTest( int canNumber ) {
  //log( F("TS\n") );
  unsigned long test = millis();
  memcpy( msgString, &test, 4 );
  cans[canNumber].sendMsgBuf( 128, 8, (INT8U*)msgString );
}

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
    if( file.open( msgString, O_CREAT | O_WRITE | O_EXCL ) ) {
      fileBytesWritten = 0;
      DateTime now = rtc.now();
      fileBytesWritten += file.print( DATE_LOG_HEADER );
      fileBytesWritten += writeLongToFile( millis() );
      fileBytesWritten += writeLongToFile( now.unixtime() );
      fileBytesWritten += file.print( NEWLINE );
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
  for( canCounter = 0; canCounter < NUM_CANS; canCounter++ ) {
    if( cans[canCounter].begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK ) {
      //cans[canCounter].setMode(MCP_LISTENONLY);
      cans[canCounter].setMode(MCP_NORMAL);
      //cans[canCounter].enOneShotTX();
      canEnabled[canCounter] = true;
      sprintf_P(msgString, PSTR("CAN %d ENABLED\n"), canCounter);
      log((char*)msgString);
    }
    else {
      sprintf_P(msgString, PSTR("CAN %d FAIL\n"), canCounter);
      log(msgString);
    }
  }
}

/* This is using an SH-HC-08 (which works on 3.3V)
 *  https://smile.amazon.com/gp/product/B01N4P7T0H
 *  http://www.dsdtech-global.com/2017/08/sh-hc-08.html
 *  https://drive.google.com/file/d/0B4urklB65vaCcEVyMm5haVVpMUk/view?usp=sharing
 *  Changes will likely be needed with other version of BLE modules
 */

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


void handleBtInput( char* input ) {
  if( !strncmp_P( input, PSTR("reset"), 5 ) ) {
    log( F("Reset command from bluetooth\n" ) );
  }
  else if( !strncmp_P( input, PSTR("stop"), 4 ) ) {
    log( F("Stop command from bluetooth\n" ) );
    loggingEnabled = false;
    switchFile();
    bt.println( F("Logging ended") );
  }
  else if( !strncmp_P( input, PSTR("start"), 5 ) ) {
    log( F("Start command from bluetooth\n" ) );
    loggingEnabled = true;
    switchFile();
    sprintf_P( msgString, PSTR( "Log enabled, %lu bytes written, filename %08d.log\n"), (unsigned long)fileBytesWritten, fileNumber );
    bt.print( msgString );
}
  else if( !strncmp_P( input, PSTR("status"), 5 ) ) {
    log( F("Status command from bluetooth\n" ) );
    if( loggingEnabled ) {
      sprintf_P( msgString, PSTR( "Log enabled, %lu bytes written, filename %08d.log\n"), (unsigned long)fileBytesWritten, fileNumber );
      bt.print( msgString );
    }
    else {
      bt.println( F("Logging disabled") );
    }
  }
  else if( !strncmp_P( input, PSTR("teststart"), 9 ) ) {
    runTest = true;
    bt.println( F("Test started\n") );
  }
  else if( !strncmp_P( input, PSTR("teststop"), 8 ) ) {
    runTest = false;
    bt.println( F("Test stopped\n") );
  }
  else {
    log( F("Received unknown bluetooth command" ) );
  }
}

void readCanMessage( MCP_CAN &can ) {
    int i;
    can.readMsgBuf( &rxId, &len, rxBuf );
    if((rxId & 0x80000000) == 0x80000000) {     // Determine if ID is standard (11 bits) or extended (29 bits)
      realRxId = rxId & 0x1FFFFFFF;
      #ifdef LOG_CANS
      sprintf_P(msgString, PSTR("%lu: (%d): Extended ID: 0x%.8lX  DLC: %1d  Data:"), (unsigned long)millis(), canCounter, realRxId, len);
      #endif
    }
    else {
      realRxId = rxId;
      #ifdef LOG_CANS
      sprintf_P(msgString, PSTR("%lu: (%d): Standard ID: 0x%.3lX       DLC: %1d  Data:"), (unsigned long)millis(), canCounter, realRxId, len);
      #endif
    }
    #ifdef LOG_CANS
    log(msgString);
    #endif
  
    isRemoteRequest = (rxId & 0x40000000) == 0x40000000; // Determine if message is a remote request frame.

    #ifdef LOG_CANS
    if( isRemoteRequest ) {    
      sprintf_P(msgString, PSTR(" REMOTE REQUEST FRAME"));
      log(msgString);
    } else {
      for(i = 0; i<len; i++){
        sprintf_P(msgString, PSTR(" 0x%.2X"), rxBuf[i]);
        log(msgString);
      }
    }
    log( F("\n") );
    #endif

    //Process Message
    switch( realRxId ) {
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
  //Log message to file
  if( loggingEnabled && fileOpen ) {
    uint8_t pos = 0;
    pos += mconcatStr( msgString, pos, CAN_MSG_LOG_HEADER, SIZE_IRRELEVANT );
    pos += mconcatLong( msgString, pos, (unsigned long)millis() );
    pos += mconcatChar( msgString, pos, (unsigned char)canCounter );
    pos += mconcatLong( msgString, pos, (unsigned long)rxId );
    pos += mconcatChar( msgString, pos, (unsigned char)len );
    pos += mconcatMem( msgString, pos, rxBuf, len );
    pos += mconcatStr( msgString, pos, NEWLINE, SIZE_IRRELEVANT );
    fileBytesWritten += file.write( (void*)msgString, (size_t)pos );
    //fileBytesWritten += file.print( CAN_MSG_LOG_HEADER );
    //fileBytesWritten += writeLongToFile( (unsigned long)millis() );
    //fileBytesWritten += file.write( (unsigned char)canCounter );
    //fileBytesWritten += writeLongToFile( (unsigned long)rxId );
    //fileBytesWritten += file.write( (unsigned char)len );
    //fileBytesWritten += file.write( rxBuf, len );
    //fileBytesWritten += file.print( NEWLINE );
  }

}

uint8_t mconcatMem( void* dest, uint8_t outPos, const void* src, uint8_t count ) {
  dest += outPos;
  memcpy( dest, src, count );
}

uint8_t mconcatChar( char* dest, uint8_t outPos, unsigned char value ) {
  dest += outPos;
  memcpy( dest, &value, sizeof( unsigned char ) );
  return sizeof( unsigned char );
}  

uint8_t mconcatLong( char* dest, uint8_t outPos, unsigned long value ) {
  dest += outPos;
  memcpy( dest, &value, sizeof( unsigned long ) );
  return sizeof( unsigned long );
}

uint8_t mconcatStr( char* dest, uint8_t outPos, char* src, uint8_t size ) {
  bool size_known = (size != SIZE_IRRELEVANT);
  uint8_t read = 0;
  dest += outPos;
  char ch = '.';
  uint8_t written = 0;
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

uint8_t mconcatStr( char* dest, uint8_t outPos, PGM_P src, uint8_t size ) {
  bool size_known = (size != SIZE_IRRELEVANT);
  const char* read = src;
  char* write = dest;
  write += outPos;
  char ch = '.';
  uint8_t written = 0;
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

int bufferedSerialRead( char* buf, int maxLen ) {
  maxLen = maxLen - 1;
  int i = 0;
  while( Serial.available() > 0 && i < maxLen) {
    buf[i++] = (char)Serial.read();
    delay( 5 );
  }
  buf[i] = 0;
  return i;
}

int bufferedRead( char* buf, int maxLen, AltSoftSerial &ser ) {
  maxLen--;
  int i = 0;
  while( ser.available() > 0 && i < maxLen) {
    buf[i++] = (char)ser.read();
    delay( 5 );
  }
  buf[i] = 0;
  return i;
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
 
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}


