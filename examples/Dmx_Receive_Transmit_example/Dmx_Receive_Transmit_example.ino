#include <Dmx_ESP32.h>

#define LED_GREEN 13
#define LED_WHITE 14

#define RX_PIN 35      // MAX485 DMX reception takes place on the UART
#define RX_RMT 35     // With the RMT peripheral looking for the 80us break
#define RX_DISABLE 33  // The pin controlling the !RE (not Read Enable) pin on the transceiver
#define TX_PIN 32
#define TX_ENABLE -1 // The pin controlling the !RE (not Read Enable) pin on the other transceiver

// Do not use the same transceiver for Rx & Tx !
// Do not use the same UART for Rx & Tx. The Tx UART will switch baud-rate to generate the break
// the RX UART can not be shifting baud-rate to receive.

#define DMX_PORT_R &Serial1
#define DMX_PORT_T &Serial2

dmxRx dmxReceive = dmxRx(DMX_PORT_R, RX_PIN, RX_RMT, RX_DISABLE, LED_GREEN, LOW);  
dmxTx dmxSend(DMX_PORT_T, TX_PIN, TX_ENABLE, LED_WHITE, LOW);

void setup() {
  Serial.begin(500000);
  Serial.println("\n");
  Serial.println("DMX Receive / Transmit");

  if (!dmxReceive.configure()) {  // configures and starts the UART (with default parameters)
    Serial.println("DMX Receive Configure failed.");
  }
  else {
    Serial.println("DMX Receive Configured.");
  }
  delay(10);
  if (dmxReceive.start()) {
    Serial.println("DMX reception Started");
  }
  else {
    Serial.println("DMX reception aborted");
  }
  
  if (!dmxSend.configure()) {
    Serial.println("DMX Transmission was previously configured.");
  }
  Serial.println("DMX Transmission initialized!");
}

void loop() {
  if (dmxReceive.hasUpdated()) {  // only read new values
    for (uint16_t i = 1; i <= DMX_FRAME; i++) {
      dmxSend.write(i, 255 - dmxReceive.read(i));  // invert all channels
    }
    // note : this could actually be done quicker by doing a 255 ^ XOR on 4 bytes at a time
    // a bitwise operation on an 8-bit variable takes just as long as on a 32-bit variable
    // it would require reading 4 bytes at a time into a uint8_t 4 byte array, and reinterpreting the pointer
    // to be a 32-bit pointer and performing the XOR operation. I bit beyond the scope of this example though.
  }
  if (dmxSend.readyToTransmit()) { 
    dmxSend.transmit();
  }
}
