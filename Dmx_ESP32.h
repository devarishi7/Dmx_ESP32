/*
	ESP32 DMX library using RNT to find the frame reset break for Rx.
	Copyright (C) 2025  Little Art Bear

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef Dmx_ESP32_h
#define Dmx_ESP32_h

#if ESP_ARDUINO_VERSION_MAJOR >= 3
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wdeprecated-declarations"  // ignore depracted warnings
		#include "Arduino.h"              // it warns for deprecation of an API that isn't used at all
	#pragma GCC diagnostic pop
#else
	#include "Arduino.h"
#endif	

#if !defined(ARDUINO_ARCH_ESP32)
	#error This library is only supported for ESP32
#endif

#define DMX_CHANNELS 513  // slot 0 and slots 1 - 512
#define DMX_FRAME 512


#define DMX_BREAK_US 150
 // should result in 60.000 = 1000000 * 9 / 150us or 133us depending on 
// whether the startbit is included. MAB will be  33us. Consider 100000 to be the Max
							// as that results in 90us break with 20us MAB

#if ESP_ARDUINO_VERSION_MAJOR >= 3
	#if ESP_ARDUINO_VERSION_MINOR < 3
		#error For ESP32 Core 3, the minimum version is 3.3.0 (Fatal errors in the RMT driver in lower versions)
	#endif
#define RMT_FREQ_HZ 1000000  // 1Mhz 1us per tick

#else

extern "C" IRAM_ATTR void idleCalback(uint32_t *data, size_t len, void * arg);  // prototype for the RMT  Idle Callback
                                                // for core 2.x.x
#endif

class dmxRx {  

  public:

    dmxRx(HardwareSerial* port, int8_t pinRx, int8_t rmtRx = -1,
          int8_t pinEnable = -1, int8_t pinToggle = -1, int8_t ledOn = HIGH,
          uint16_t breakLength = 80, uint8_t filt = 0); 

    bool configure();   
    bool start(bool enable = true);
    bool stop(); 
    void rxDisable(bool ledOff = true);
    void rxEnable();
	bool rxIsEnabled();
    uint8_t* dmxBuffer();
	bool updateAvailable();
    bool hasUpdated(uint16_t timeOut = 0);
	bool hasUpdatedChannel(uint16_t channel, uint16_t timeOut = 50);
    void resetUpdated();	
    uint8_t read(uint16_t ch);
    uint16_t readBytes(uint8_t* data, uint16_t numBytes, uint16_t startChannel);
	bool readBytesWhenAvailable(uint8_t* data, uint16_t numBytes, uint16_t startChannel, 
	                            bool startWithBreak = true, uint16_t timeOut = 50);
	bool readBytesWhenFoundBreak(uint8_t* data, uint16_t numBytes, uint16_t startChannel, uint16_t timeOut = 50);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
	size_t rx_num_symbols = RMT_MEM_NUM_BLOCKS_1 * RMT_SYMBOLS_PER_CHANNEL_BLOCK;
	rmt_data_t rx_symbols[RMT_MEM_NUM_BLOCKS_1 * RMT_SYMBOLS_PER_CHANNEL_BLOCK];
    inline void breakDetected(void *arg);
	int8_t rmtRxPin();  
#else
    IRAM_ATTR void breakDetected(); 
#endif

  private: 
    
    static const uint16_t _maxChannels = DMX_CHANNELS + 1;  
#if ESP_ARDUINO_VERSION_MAJOR >= 3
	inline void toggleLed();
	TaskHandle_t _rmtReadTaskHandle = NULL;
#if ESP_ARDUINO_VERSION_PATCH > 0
	int8_t _rmtRxPinBusNr, _rmtRxChannelNr, _uartRxPinBusNr, _uartRxChannelNr; 
	void * _rmt_bus = NULL;
	void * _uart_bus = NULL;
#endif
#else
	IRAM_ATTR inline void toggleLed();
    rmt_obj_t* _rmt_recv = NULL;    
#endif
    HardwareSerial* _dmxPort = NULL;
	uint8_t _dmxBuf[_maxChannels];
    int8_t _pinRx, _rmtRx, _pinEnable, _pinToggle, _ledOn, _pinTx;	
    uint16_t _breakLength, _filter;
    bool _configured = false, _hasStarted = false, _isEnabled = false;
    volatile bool _foundBreak = false, _firstFrame = true;
    volatile uint16_t _readAlready = 0; 	
};  // class dmxRx


class dmxTx {
	
  public:

    dmxTx(HardwareSerial* port, int8_t pinTx, int8_t pinEnable = -1, int8_t pinToggle = -1, int8_t ledOn = HIGH);
	bool configure();
	bool write(uint8_t data, uint16_t channel);
	uint16_t writeBytes(uint8_t* data, uint16_t numBytes, uint16_t startChannel = 1);
	uint8_t* dmxBuffer();
	bool transmit();
	bool transmitBreak();  // transmits only the break to allow for multiple outputs to run in sync (and not waste time)
	bool readyToTransmit();
	const uint32_t transmitMicros();
	uint32_t breakLength(bool withMAB = false);
	void setBreakLength(uint32_t length_us);

  private:	
#if ESP_ARDUINO_VERSION_MAJOR >= 3
	inline void toggleLed();
#else
	IRAM_ATTR inline void toggleLed();
#endif
	HardwareSerial* _dmxPort = NULL;
	int8_t _pinEnable, _pinToggle, _ledOn, _pinTx, _pinRx;
	uint8_t _dmxBuf[DMX_CHANNELS];
	uint16_t _writeSize;
	bool _configured = false, _sendBreakFirst = true;
	uint32_t _breakBaud = (1000000 * 9 / DMX_BREAK_US);
};

#endif // #ifndef Dmx_ESP32_h