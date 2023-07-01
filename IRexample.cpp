#include <Arduino.h>
#include <IRremote.hpp>
#define RECV_PIN 13
#define OUT 27 
void setup()
{
	IrReceiver.begin(RECV_PIN);
  	Serial.begin(9600);
  	Serial.println("Enabling IRin");
  	Serial.println("Enabled IRin");
}
void loop() {
  if (IrReceiver.decode()) {
      Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
    // USE NEW 3.x FUNCTIONS
    //   IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
    //   IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data
      IrReceiver.resume(); // Enable receiving of the next value
  }
  delay(100);
}