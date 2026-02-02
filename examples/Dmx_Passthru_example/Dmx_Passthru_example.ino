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
  Serial.println("DMX Passthru.");

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
  Serial.print("Default DMX Transmit Breaklength is set to ");
  Serial.print(dmxSend.breakLength(), DEC); 
  Serial.println(" us.");
  
  dmxSend.setBreakLength(90);
  
  Serial.print("Custom DMX Transmit Breaklength set to ");
  Serial.print(dmxSend.breakLength(), DEC); 
  Serial.println(" us.");
  
  if (!dmxSend.configure()) {
    Serial.println("DMX Transmission was previously configured.");
  }
  Serial.println("DMX Transmission initialized!");
}

void loop() {
  if (dmxReceive.hasUpdated()) {  // only read new values
    // Using the reference to the receive DMX buffer is the quickest and most memory efficient
    dmxSend.writeBytes(dmxReceive.dmxBuffer(), 512);
    // but reading like that doesn't reset the hasUpdated() flag
    dmxReceive.resetUpdated();
    // Alternatively you can do a normal read on a single channel, which will also reset the flag
  }

  if (dmxSend.readyToTransmit()) { 
    dmxSend.transmit();
  }
}
