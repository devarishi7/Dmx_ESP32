#include <Dmx_ESP32.h>

#define LED_GREEN 13
#define LED_WHITE 14

#define TX_PIN 32
#define TX_ENABLE -1 // there is no real benefit in using an actual pin to do this
                     // unlike for reception where disabling input stops the RMT & UART 
                     // interrupt callbacks from being called. The DE can be hardwired to 
                     // Vcc of the transceiver

#define DMX_PORT &Serial2

dmxTx dmxSend(DMX_PORT, TX_PIN, TX_ENABLE, LED_GREEN, LOW);

void setup() {
  Serial.begin(500000);
  Serial.println("\n");
  Serial.println("DMX Transmission");
  if (!dmxSend.configure()) {
    Serial.println("DMX Tx was previously configured.");
  }
  Serial.println("DMX initialized!");
  pinMode(LED_WHITE, OUTPUT);
}

void loop() {
  static uint8_t count = 0;

  if (dmxSend.readyToTransmit()) {  // readyToTransmit uses the available space in the 
                                   // Tx buffer + Tx FIFO as a reference set during configure
  //if (1) { // uncomment this line and comment the previous one to confirm it works as intended
  
    digitalWrite(LED_WHITE, HIGH);  // turns 'off' the LED
    dmxSend.transmit();  // transmit blocks for the duration of the previous transmission
                         // plus the duration of the break. 
                         
    count++;      
    dmxSend.write(count, 7);    
  }
  digitalWrite(LED_WHITE, LOW);  // turns 'on' the LED
}
