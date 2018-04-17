#include <mcp_can.h>

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

MCP_CAN can(2);
boolean canEnabled = true;
unsigned long lastSend = 0;
unsigned long msgNumber = 0;
INT8U msgString[64];
#define SEND_INTERVAL_MICROS 800
#define REPORT_INTERVAL_MILLIS 5000
unsigned long lastReport = 0;
void setup() {
  Serial.begin(500000);
  Serial.print( F("Setting up\n") );
  if( can.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK ) {
    can.setMode(MCP_NORMAL);
    can.enOneShotTX();
    canEnabled = true;
    Serial.println( F("Can enabled") );
  }

}

void loop() {
  if( micros() - lastSend > SEND_INTERVAL_MICROS ) {
    unsigned long test = micros();
    char *buf = msgString;
    memcpy( buf, &msgNumber, 4 );
    buf += 4;
    memcpy( buf, &test, 4 );
    if( can.sendMsgBuf( 128, 8, msgString ) != CAN_OK ) {
      Serial.print( F("Error sending message") );
    }
    else {
      msgNumber++;
    }
    lastSend = micros();
  }
  if( millis() - lastReport > REPORT_INTERVAL_MILLIS ) {
    lastReport = millis();
    Serial.println( msgNumber );
  }
}
