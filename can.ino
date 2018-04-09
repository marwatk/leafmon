#include <SdFat.h>
#include <AltSoftSerial.h>
#include <mcp_can.h>
//#include <SoftwareSerial.h>

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


/*
 * SD Card
 * CS - 10
 * SO - 12
 * SI - 11
 * SCK - 13
 */
//25ma at 3.3V

MCP_CAN cans[3] = { //CS pins
  MCP_CAN(2),
  MCP_CAN(4),
  MCP_CAN(6),
};

int canInterupts[] = {3, 5, 7};
bool canEnabled[] = {false,false,false};
char canNames[] = "ECA";

int canCounter = 0;
#define NUM_CANS 3


#define BT_POWER_PIN A3
//#define CAN0_INT 7 //CAN INT
//MCP_CAN CAN0(10); //CAN CS

int read;
long unsigned int rxId;
long unsigned int realRxId;
unsigned char len = 0;
unsigned char rxBuf[8];
bool isRemoteRequest = false;

unsigned char odo1;
unsigned char odo2;
unsigned char odo3;
unsigned char SOH;
unsigned char ActiveFuelBars;

#define BUF_SIZE 64
char miscBuf[BUF_SIZE];
unsigned long nextReport = 0;
unsigned long reportInterval = 5000;

#define MAX_FILE_SIZE 10485760 //10MB
#define FLUSH_FILE_BYTES 4096
boolean loggingEnabled = false;
unsigned int fileNumber = 0;
unsigned int fileBytesWritten = 0;
unsigned int fileBytesWrittenSinceFlush = 0;
boolean fileOpen = false;

int batteryVolts12V = 0;

//SoftwareSerial bt( A1, A2 ); //RX, TX
AltSoftSerial bt;
SdFat sd;

SdFile file;

void openNextFile() {
  if( fileOpen ) {
    
  }
  if( loggingEnabled ) {
    //do {
    //  sprintf( miscBuf, "L_%06d.log", fileNumber );
    //  fileNumber++;
    //} while( SD.exists( miscBuf ) );
  }
}

void setup() {
  Serial.begin(500000);
  Serial.write( "Setting up\n" );
  //pinMode( BT_POWER_PIN, OUTPUT );
  //setBtPower( true );
  delay( 200 ); //Wait for BT to initialize
  setupBt();

  sprintf( miscBuf, F("This is a flash-based message with an argument %d"), 100 );
  log( miscBuf );
  //bt.println("BT Started");
  //bt.listen();
  for( canCounter = 0; canCounter < NUM_CANS; canCounter++ ) {
    if( cans[canCounter].begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK ) {
      cans[canCounter].setMode(MCP_LISTENONLY);
      canEnabled[canCounter] = true;
      sprintf(miscBuf, "CAN %d ENABLED\n", canCounter);
      log(miscBuf);
    }
    else {
      sprintf(miscBuf, "CAN %d FAIL\n", canCounter);
      log(miscBuf);
    }
  }
  /*
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    canEnabled = true;
    log("MCP Ok\n");
    CAN0.setMode(MCP_LISTENONLY);
    pinMode(CAN0_INT, INPUT);
  }
  else {
    log("MCP KO\n");
  }
  */
}

/* This is using an SH-HC-08 (which works on 3.3V)
 *  https://smile.amazon.com/gp/product/B01N4P7T0H
 *  http://www.dsdtech-global.com/2017/08/sh-hc-08.html
 *  https://drive.google.com/file/d/0B4urklB65vaCcEVyMm5haVVpMUk/view?usp=sharing
 *  Changes will likely be needed with other version of BLE modules
 */

void setupBt() {
  bt.begin(9600); //Defaults to 9600
  sendBtAtCommand( "AT+NAMELeafLogger" );
  readBtCommandResponse();
  sendBtAtCommand( "AT+BAUD6" ); //4=9600, 5=19200, 6=38400, 7=57600, 8=115200
  readBtCommandResponse();
  bt.begin(38400);
  sendBtAtCommand( "AT" );
  readBtCommandResponse();
}

void sendBtAtCommand( char* command ) {
  Serial.print( "BT Command: " );
  Serial.print( command );
  Serial.print( "\n" );
  bt.write( command );
  bt.write( "\n" );
}
void readBtCommandResponse() {
  delay( 100 );
  while( (read = bufferedRead( buf, BUF_SIZE, bt )) > 0 ) {
    log( "BT  Output: " );
    log( buf );
    log( "\n" );
    delay( 10 );
  }
}

/*
void setBtPower( boolean power ) {
  if( power ) {
    digitalWrite( BT_POWER_PIN, HIGH );
  }
  else {
    digitalWrite( BT_POWER_PIN, LOW );
  }
}
*/

void log( char b ) {
  //bt.write( b );
  Serial.write( b );
}

void log( char* b ) {
  int i = 0;
  while( b[i] != 0 && i < 255 ) {
    log( b[i++] );
  }
}

void loop() {
  for( canCounter = 0; canCounter < NUM_CANS; canCounter++ ) {
    if( !digitalRead(canInterupts[canCounter]) && canEnabled[canCounter] ) {
      readCanMessage( cans[canCounter] );
    }
  }
  //if( !digitalRead(CAN0_INT) && canEnabled ) {
  //  readCanMessage( CAN0 );
  //}
  while( (read = bufferedRead( buf, BUF_SIZE, bt )) > 0 ) {
    log( buf );
  }
  while( (read = bufferedSerialRead( buf, BUF_SIZE )) > 0 ) {
    sendBtAtCommand( buf );
    readBtCommandResponse();
    //log( buf );
  }
  if( millis() > nextReport ) {
    report();
    nextReport = millis() + reportInterval;
  }

}

void readCanMessage( MCP_CAN &can ) {
    int i;
    can.readMsgBuf( &rxId, &len, rxBuf );
    if((rxId & 0x80000000) == 0x80000000) {     // Determine if ID is standard (11 bits) or extended (29 bits)
      realRxId = rxId & 0x1FFFFFFF;
      sprintf(miscBuf, "%c: Extended ID: 0x%.8lX  DLC: %1d  Data:", canNames[canCounter], realRxId, len);
    }
    else {
      realRxId = rxId;
      sprintf(miscBuf, "%c: Standard ID: 0x%.3lX       DLC: %1d  Data:", canNames[canCounter], realRxId, len);
    }
    
    log(miscBuf);
  
    isRemoteRequest = (rxId & 0x40000000) == 0x40000000; // Determine if message is a remote request frame.
    if( isRemoteRequest ) {    
      sprintf(miscBuf, " REMOTE REQUEST FRAME");
      log(miscBuf);
    } else {
      for(i = 0; i<len; i++){
        sprintf(miscBuf, " 0x%.2X", rxBuf[i]);
        log(miscBuf);
      }
    }
    log( "\n" );
    processCanMessage();
}

void logCanMessage() {
  
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
  maxLen = maxLen - 1;
  int i = 0;
  while( ser.available() > 0 && i < maxLen) {
    buf[i++] = (char)ser.read();
    delay( 5 );
  }
  buf[i] = 0;
  return i;
}

void report() {
  log( "Report\n" );
}

void processCanMessage() {
  boolean doLog = true;
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
  if( doLog ) {
    logCanMessage();
  }
}


