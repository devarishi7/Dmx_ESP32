#include <Dmx_ESP32.h>

#define LED_GREEN 13

#define RX_PIN 35      // MAX485 DMX reception takes place on the UART
#define RX_RMT 35     // With the RMT peripheral looking for the 80us break
                      // you can either define 2 separate pins or choose the same pin
                      // 2 Separate pins should be connected through a current limiting resistor only

#define RX_DISABLE 33  // The pin controlling the !RE (not Read Enable) pin on the transceiver
                       // DMX Rx & Tx should not be done on the same transceiver (RDM is not supported)
                       // DE should be hardwired to GND. 
#define DMX_PORT &Serial1  

#define CHANNELS 6

uint8_t dmxValue[CHANNELS];

const uint16_t dmxChannel = 12;

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
}

void loop() {
  static bool receiving = false;
  if (dmxReceive.readBytesWhenAvailable(dmxValue, CHANNELS, dmxChannel)) {  // blocking and waiting for the 
                // reception of the channels specified. Optiobnally a timeOut can be 
                // specified. It defaults to 50ms. A DMX frame will take almost 23ms to transmit
                // if you expect your other processing to take longer than that occasionally, you should 
                // use readBytesWhenFoundBreak() instead. There is no need to wait for a frame to become
                // available, you can read and all values wiil be updated when the frame reset break
                // is found. If you wait till the break is found, the test for the oversize frame is done as well
                // testing for frame that does not contain all channels is pointless since the spec allows for incomplete
                // frames. More Functions are available for certain purposes. Please check the header
                // file for the syntax.
    
    if (!receiving) {
      Serial.println("Receiving DMX");
      receiving = true;
    }
    doWithChannels();
  }
  else {
    if (receiving) {
      Serial.println("DMX reception stopped");
      receiving = false;
    }
  }
  otherProcessing();
}

void otherProcessing() {
  // whatever other processing you want to do
}

void doWithChannels() {
  // whatever you want to do with them
}

