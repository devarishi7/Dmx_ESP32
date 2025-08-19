#include <Dmx_ESP32.h>

#define LED_GREEN 13

#define LED_WHITE 14
#define INVERT_LED 255 -  // if the LED is connected 'active LOW' this will invert the output

#define RX_PIN 35      // MAX485 DMX reception takes place on the UART
#define RX_RMT 35     // With the RMT peripheral looking for the 80us break
                      // you can either define 2 separate pins or choose the same pin
                      // 2 Separate pins should be connected through a current limiting resistor only

#define RX_DISABLE 33  // The pin controlling the !RE (not Read Enable) pin on the transceiver
                       // DMX Rx & Tx should not be done on the same transceiver *RDM is not supported)
                       // DE should be hardwired to GND. 
#define DMX_PORT &Serial1  

dmxRx dmxReceive = dmxRx(DMX_PORT, RX_PIN, RX_RMT, RX_DISABLE, LED_GREEN, LOW);  // the toggle LED is
                                                             // connected 'active LOW' in this situation

/*  Declaration of the construtor in 'DMX_esp32.h'
 *  dmxRx(HardwareSerial* port, int8_t pinRx, int8_t rmtRx = -1,
          int8_t pinEnable = -1, int8_t pinToggle = -1, int8_t ledOn = HIGH,
          uint16_t breakLength = 80, uint8_t filt = 0); */

void setup() {
  Serial.begin(500000);
  Serial.println("\n");
  Serial.println("DMX Reception using RMT for break detection.");

  if (!dmxReceive.configure()) {  // configures and starts the UART (with default parameters)
    Serial.println("DMX Configure failed.");
  }
  else {
    Serial.println("DMX Configured.");
  }
  delay(10);
  if (dmxReceive.start()) {
    Serial.println("DMX reception Started");
  }
  else {
    Serial.println("DMX aborted");
  }
  pinMode(LED_WHITE, OUTPUT);  // the indicator LED
}

void loop() {
  if (dmxReceive.hasUpdated()) {  // only read new values
    analogWrite(LED_WHITE, INVERT_LED dmxReceive.read(7));  // the LED i have is active LOW
  }
}
