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

#include "Dmx_ESP32.h"

#define DMX_BREAK_BAUD 60000  // should result in 60.000 = 1000000 * 9 / 150us or 133us depending on 
							// whether the startbit is included. MAB will be  33us. Consider 100000 to be the Max
							// as that results in 90us break with 20us MAB
#define SERIAL_SIZE_TX 520
#define DMX_BAUD 250000
#define DMX_FORMAT SERIAL_8N2
#define SERIAL_SIZE_RX 520  // in theory only 514 bytes are required, but it is not such a scarce resource
#define MAX_TIMEOUT 150  // ms


#if ESP_IDF_VERSION_MAJOR >= 5

#define READ_TASK_STACK_SIZE 3072  // i thought 1536 was enough, but it's not.
#define READ_TASK_PRIORITY 4       // default from Callback example is 4

#include "esp32-hal-periman.h"  

//---------------------------------------- Rx --------------------------------------------

// Dummy bus detach callback functions

static bool _rmtRXPinDetachBus(void *busptr) {
  // does nothing and keeps the RMT driver running attached to RX pin
  log_d("_rmtRXPinDetachBus called");
  return true;
}

static bool _uartRXPinDetachBus(void *busptr) {
  // does nothing and keeps the UART driver running attached to RX pin
  log_d("_uartRXPinDetachBus called");
  return true;
}

// RMT readTask.

static void readTask(void *arg) 
{
	dmxRx* p = (dmxRx*)arg;
	while (1) {
		rmtRead(p->rmtRxPin(), p->rx_symbols, &p->rx_num_symbols, RMT_WAIT_FOR_EVER);  
		p->breakDetected(p);
	}
	vTaskDelete(NULL);
}

#endif

// dmxRx Contructor, pinRx & rmtRx can be the same pin in core 2.x.x
// And also in core >= 3.3.0 thanks to @SuGlider who showed how to disable the bus detach for the RMT & UART
// The method does disable bus detachment on those 2 peripherals for the rest of the program. So if one intends to 
// Re use any pin that has been assigned to any RMT-Rx channel or UART-RX before, then use 2 separate pins and
// conmect them through a current limiting resistor
// The Enable pin should be connected to !RE only and not be used as a direction pin. DE should be connected to GND
// The Toggle Pin can have a LED connected, to show incoming frames (it will toggle on the found break) 
// Either active HIGH or active LOW
// Breaklength defaults to the spec 80us and the RMT filter is normally disabled

dmxRx::dmxRx(HardwareSerial* port, int8_t pinRx, int8_t rmtRx,
			int8_t pinEnable, int8_t pinToggle, int8_t ledOn,
			uint16_t breakLength, uint8_t filt) 
{  
	_dmxPort = port;
	_pinRx = pinRx;
	if (rmtRx == -1) {  // defaults to -1 which means just 1 pin for both
		_rmtRx = pinRx;
	}
	else {
		_rmtRx = rmtRx;
	}
	_pinTx = -1;
	_pinEnable = pinEnable;
	_pinToggle = pinToggle;
	_ledOn = ledOn;
	_breakLength = breakLength;
	_filter = filt;
	  
      // the RMT needs to be initalized in the constructor.
#if ESP_IDF_VERSION_MAJOR >= 5  // The RNT API in core 3 is competely different from core 2
                               
	if (!rmtInit(_rmtRx, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_1, RMT_FREQ_HZ)) {
		log_e("RMT not initialized");
		return;
	}
	if (!rmtSetRxMaxThreshold(_rmtRx, _breakLength)) {
		log_e("Breaklength not set");
	}
	if (!rmtSetRxMinThreshold(_rmtRx, _filter)) {
		log_e("Filter not set");  // the function returned an error
		// Doesn't mean the filter is on or off.
	}
#else
	
	if ((_rmt_recv = rmtInit(_rmtRx, RMT_RX_MODE, RMT_MEM_64)) == NULL) {
		log_e("RMT not initialized");
	}
	float realNanoTick = rmtSetTick(_rmt_recv, 1000);  // discard the value
	if (rmtSetRxThreshold(_rmt_recv, _breakLength) != ESP_OK) {
		log_e("Breaklength not set");
	}

	bool enableFilter = false;
	if (_filter) enableFilter = true;
	if (rmtSetFilter(_rmt_recv, enableFilter, _filter) != ESP_OK) {
		log_e("Filter not set");  // the function returned an error
	// Doesn't mean the filter is on or off.
	}
#endif
}

bool dmxRx::configure() 
{  // call this in setup()

#if ESP_IDF_VERSION_MAJOR >= 5
	void *rmtBus = NULL;
	if (_pinRx == _rmtRx) {  // If the pins are the same use the peripheral manager to allow the pin to be shared
		log_e("Using the same pin for UART & RMT in core v3 disables bus detach for UART Rx & RMT Rx");
		rmtBus = perimanGetPinBus(_rmtRx, ESP32_BUS_TYPE_RMT_RX);  // finds the bus 
		if (rmtBus ==  NULL) {                                    // this method is thanx to @SuGlider 
			log_e("Error aquiring the RMT Bus Peripheral Manager information");
		}
		perimanSetBusDeinit(ESP32_BUS_TYPE_RMT_RX, _rmtRXPinDetachBus);  // Attaches the dummy detachpin callback
	}
#endif

	_dmxPort->setRxBufferSize(SERIAL_SIZE_RX);              // Make sure the RX buffer is big enough.
	_dmxPort->begin(DMX_BAUD, DMX_FORMAT, _pinRx, _pinTx);  // Start the UART
	_dmxPort->setRxFIFOFull(1);                // one byte at a time to make sure the break is in the right spot
	
#if ESP_IDF_VERSION_MAJOR >= 5
	if (_pinRx == _rmtRx) { 
		perimanSetBusDeinit(ESP32_BUS_TYPE_UART_RX, _uartRXPinDetachBus);  // attach the other dummy callback
		if (!perimanSetPinBus(_rmtRx, ESP32_BUS_TYPE_RMT_RX, rmtBus, -1, -1)) {  // Set the pin Bus Type to RMT again
			log_e("Can't allocate the GPIO %d for RMT RX in the Peripheral Manager.", _rmtRx);
		}                          // or the RMT reception won't start (when the pin is set to UART
		log_w("If you intend to detach these busses at some point, use 2 separate pins");
	}
#endif

	pinMode(_pinEnable, OUTPUT);
	digitalWrite(_pinEnable, HIGH); // not receive mode. The library expects the pin to be
								// wired to the !RE pin only, basically leaving it unenabled unless used
								// if the !RE & DE pins are bridged, you should switch the pins manually
	pinMode(_pinToggle, OUTPUT); // Never have 2 transceivers in DE mode at the same time !
	digitalWrite(_pinToggle, _ledOn);
	_configured = true;
	return true;
}

bool dmxRx::start(bool enable) 
{
	if (!_configured) {  // fails, we can only start when configured
		return false;
	}
#if ESP_IDF_VERSION_MAJOR >= 5
	xTaskCreate(readTask, "BreakDetector", READ_TASK_STACK_SIZE, this, READ_TASK_PRIORITY, NULL);  // note the priority of the task
#else
	rmtRead(_rmt_recv, idleCalback, this);
#endif
	uint16_t rest = _dmxPort->available();
	uint8_t dummy[rest];    // the rx Buffer can also be flushed with .flush(false)
	_dmxPort->read(dummy, rest);  //  but that also flushes the tx buffer which may consume time.
	                              // the FIFO should be empty

	if (enable) {   // we may want to start without enabling. enable defaults to true
		_isEnabled = true;
		digitalWrite(_pinEnable, LOW); // receive mode
	}
	_firstFrame = true;  // we will discard the first frame before we have found a break
	return true;
}



bool dmxRx::stop() 
{
	if (!_configured) {
		return false;
	}
	_configured = false;  // to re-install you will need to re-configure
	_dmxPort->end();  // end the Serial port
#if ESP_IDF_VERSION_MAJOR >= 5
	rmtDeinit(_rmtRx); // end RMT
#else
	rmtDeinit(_rmt_recv); // end RMT
	_rmt_recv = NULL; // set the pointer to NULL
#endif
	digitalWrite(_pinEnable, HIGH);  // disable the transceiver
	_isEnabled = false;
	digitalWrite(_pinToggle, HIGH - _ledOn);  // turn off the toggle LED
	return true;
}

void dmxRx::rxDisable(bool ledOff)   // ledOff defaults to 'true'
{                                   // basically if the RX-pin is on the transceiver is disabled
	digitalWrite(_pinEnable, HIGH); // the RMT callback doesn't get called
	_isEnabled = false;             // nor does the Serial reception 
	if (ledOff) digitalWrite(_pinToggle, HIGH - _ledOn);  // turn off the toggle LED
}                               

void dmxRx::rxEnable() 
{
      digitalWrite(_pinEnable, LOW);
	  _isEnabled = true;
      _firstFrame = true;  // always skip the first frame. 
	  // Until we have found the break we can not be sure about the channel
}

bool dmxRx::rxIsEnabled() 
{
	return _isEnabled;
}


uint8_t* dmxRx::dmxBuffer() 
{
	return _dmxBuf;  // the pointer to the buffer allows direct reference 
}

bool dmxRx::hasUpdated(uint16_t timeOut) 
{
    if (timeOut) {
		if (timeOut > MAX_TIMEOUT) {
			timeOut = MAX_TIMEOUT;
		}
		uint32_t moment = millis(); 
		while ((!_foundBreak) && (millis() - moment <= timeOut)) {
			yield();
		}	
	}
	return _foundBreak;
}

void dmxRx::resetUpdated()  // when reading through a direct reference the updated flag needs to be reset manually
{
	_foundBreak = false;
}

uint8_t dmxRx::read(uint16_t ch)  // read a single channel from the buffer
{
    _foundBreak = false;  // we are reading, so we need to reset this flag
    if (ch < DMX_CHANNELS) {
		return _dmxBuf[ch];
	}
    return 0;
}

uint16_t dmxRx::readBytes(uint8_t* data, uint16_t numBytes, uint16_t startChannel) // read multiple channels
{
	_foundBreak = false;  // we are reading, so we need to reset this flag
	if (startChannel + numBytes < _maxChannels) {  
		uint8_t* startPtr = _dmxBuf + startChannel;
		memcpy(data, startPtr, numBytes);
	}
	else if (startChannel < _maxChannels) {
		numBytes = _maxChannels - startChannel;
		uint8_t* startPtr = _dmxBuf + startChannel;
		memcpy(data, startPtr, numBytes);
	}
	else {
		return 0;
	}
	return numBytes;
}


// Instead of waiting for the whole frame to have been received, start reading the moment the last channel has been
// updated. Reducing 'lag' which otherwise could be over 20ms for the first few channels of the frame.
// (a whole frame of 513 bytes = 513 * 44 us == 22.572 ms  + the break time.) 
// The bytes in the dmx buffer get copied from the UART rx-buffer automatically when a break is received, but
// there is no need to wait. When the last channel to be read is close to the end of the frame, using the 
// readBytesWhenFoundBreak() is probably the better option, to prevent the foundBreak function
// and this one from interfering with each other.
bool dmxRx::readBytesWhenAvailable(uint8_t* data, uint16_t numBytes, uint16_t startChannel, 
								   bool startWithBreak, uint16_t timeOut) 
{
	if (timeOut > MAX_TIMEOUT) {
		timeOut = MAX_TIMEOUT;
	}
	if ((startChannel < 1) || (startChannel > DMX_CHANNELS - 1) || (numBytes < 1) || (numBytes > DMX_CHANNELS - 1)) { 
		return false;  // the 512 maximum probably fails anyway
	}
	uint16_t lastChannel = startChannel + numBytes - 1;  // eg read startChannel = 1, numbytes = 3, lastChannel = 3
	if (lastChannel > DMX_CHANNELS - 1) {
		return false;
	}
	if (hasUpdatedChannel(lastChannel, timeOut)) {
		memcpy(data, _dmxBuf + startChannel, numBytes);
		return true;
	}
	return false;  // Time Out
}

bool dmxRx::hasUpdatedChannel(uint16_t channel, uint16_t timeOut) {
	if (timeOut > MAX_TIMEOUT) {
		timeOut = MAX_TIMEOUT;
	}
	if (channel > DMX_CHANNELS - 1) {
		return false;
	}
	uint32_t moment = millis(); 
	while(millis() - moment <= timeOut) {
		if (!_firstFrame) {
			uint16_t channelsAvailable = _dmxPort->available();
			if ((channel > _readAlready) && (channelsAvailable + _readAlready > channel)) {	
				if (channelsAvailable + _readAlready > _maxChannels) {  // buffer overflow
					_dmxPort->read(_dmxBuf + _readAlready, _maxChannels - _readAlready);
					uint16_t rest = channelsAvailable - (_maxChannels - _readAlready);
					uint8_t dummy[rest];
					_dmxPort->read(dummy, rest);
					_readAlready = 0; // maxChannels have been read.
					_firstFrame = true; //obviously something has gone wrong, wait for a break
				}
				else {
					_dmxPort->read(_dmxBuf + _readAlready, channelsAvailable);  // read what is available
					_readAlready += channelsAvailable;  // count what we've read
					return true;
				}				
			}
		}
		yield();		
	}
	return false;	
}

bool dmxRx::updateAvailable() {
	if (!_firstFrame) {
		uint16_t channelsAvailable = _dmxPort->available();
		if ((channelsAvailable) || (_foundBreak)) {
			if (channelsAvailable + _readAlready > _maxChannels) {  // buffer overflow
				_dmxPort->read(_dmxBuf + _readAlready, _maxChannels - _readAlready);
				uint16_t rest = channelsAvailable - (_maxChannels - _readAlready);
				uint8_t dummy[rest];
				_dmxPort->read(dummy, rest);
				_readAlready = 0; // maxChannels have been read.
				_firstFrame = true; //obviously something has gone wrong, wait for a break
			}
			else {
				_dmxPort->read(_dmxBuf + _readAlready, channelsAvailable);  // read what is available
				_readAlready += channelsAvailable;  // count what we've read
				return true;
			}
		}
	}
	return false;
}

// Waits for the break and copies the requested channels.

bool dmxRx::readBytesWhenFoundBreak(uint8_t* data, uint16_t numBytes, uint16_t startChannel, uint16_t timeOut) 
{
	if (timeOut > MAX_TIMEOUT) {
		timeOut = MAX_TIMEOUT;
	}
	if ((startChannel < 1) || (startChannel > DMX_CHANNELS - 1) || (numBytes < 1) || (numBytes > DMX_CHANNELS - 1)) { 
		return false; 
	}
	if (startChannel + numBytes > DMX_CHANNELS) {
		return false;
	}
	if (hasUpdated(timeOut)) {
		if (readBytes(data, numBytes, startChannel) == numBytes) {
			return true;
		}
	}
	return false;
}

#if ESP_IDF_VERSION_MAJOR >= 5

int8_t dmxRx::rmtRxPin()
{
	return _rmtRx;
}

inline void dmxRx::breakDetected(void *arg) 
{
	dmxRx* p = (dmxRx*)arg;
	p->_foundBreak = true;
	if (p->_firstFrame) {  // flush the buffer
		p->_firstFrame = false;
		uint16_t rest = p->_dmxPort->available();
		if (rest) {
			uint8_t dummy[rest];  // first frame gets flushed, but flush() just doesn't work properly
			p->_dmxPort->read(dummy, rest);
		}
	}
	else {
		uint16_t channels = p->_dmxPort->available();
		if (channels) {
			if (channels + p->_readAlready > p->_maxChannels) {  // we only discard any excess channels, we do still
				/*p->_dmxPort->read(p->_dmxBuf + p->_readAlready, p->_maxChannels - p->_readAlready); // copy them
				uint16_t rest = channels - (p->_maxChannels - p->_readAlready); // we could discard all of them 
				uint8_t dummy[rest];
				p->_dmxPort->read(dummy, rest);*/
				uint8_t dummy[channels];   // it is better to discard an oversize packet
				p->_dmxPort->read(dummy, channels);
			}
			else {    // the _readAlready variable keeps track of how many bytes have been transferred from
				p->_dmxPort->read(p->_dmxBuf + p->_readAlready, channels);
			}         // the rx-buffer to the dmx buffer already
			p->toggleLed();  // if there are channels and it's not the first frame. Toggle the led
		}
	}
	p->_readAlready = 0;  // all has been read
}

inline void dmxRx::toggleLed() 
{  
	static bool ledOn = false;
	ledOn = !ledOn;
	if (ledOn) digitalWrite(_pinToggle, HIGH);
	else digitalWrite(_pinToggle, LOW);
}

#else

IRAM_ATTR void dmxRx::breakDetected() 
{
	if (_firstFrame) {
		_firstFrame = false;
		uint16_t rest = _dmxPort->available();
		if (rest) {
			uint8_t dummy[rest];  // first frame gets flushed
			_dmxPort->read(dummy, rest);
		}
	}
	else {
		uint16_t channels = _dmxPort->available();
		if (channels) {
			if (channels + _readAlready > _maxChannels) {
				/*_dmxPort->read(_dmxBuf + _readAlready, _maxChannels - _readAlready);
				uint16_t rest = channels - (_maxChannels - _readAlready);
				uint8_t dummy[rest];
				_dmxPort->read(dummy, rest);*/
				uint8_t dummy[channels];
				_dmxPort->read(dummy, channels);
			}
			else {
				_dmxPort->read(_dmxBuf + _readAlready, channels);
			}
			toggleLed();
		}		
	}
	_foundBreak = true;
	_readAlready = 0;
}

IRAM_ATTR inline void dmxRx::toggleLed() 
{  // called from an IRAM_ATTR function
	static bool ledOn = false;
	ledOn = !ledOn;
	if (ledOn) digitalWrite(_pinToggle, HIGH);
	else digitalWrite(_pinToggle, LOW);
}

IRAM_ATTR void idleCalback(uint32_t *data, size_t len, void * arg) {  // this is the callback
	dmxRx* p = (dmxRx* )arg;
	p->breakDetected();
}

#endif

//------------------------------------- Tx --------------------------------------------------------

dmxTx::dmxTx(HardwareSerial* port, int8_t pinTx, int8_t pinEnable, int8_t pinToggle, int8_t ledOn)
{
	_dmxPort = port;
	_pinTx = pinTx;
	_pinRx = -1;  // it just isn't used, the switching of the baudrate for the break generation 
	_pinEnable = pinEnable;  // makes it impossible to do Rx & Tx on the same UART at the same time
	_pinToggle = pinToggle;  // RDM is not supported within this library
	_ledOn = ledOn;
}

bool dmxTx::configure() 
{
	if (_configured) {		
		return false;
	}
	_dmxPort->setTxBufferSize(SERIAL_SIZE_TX); // The transmit function is blocking while anything written
			// to the Serial port needs to be transferred to the tx buffer. Once that is completed the processor can
			// continue and interrupts will move blocks to the FIFO, so it makes sense to have it the size of the frame
	_dmxPort->begin(DMX_BAUD, DMX_FORMAT, _pinRx, _pinTx);
	_writeSize = _dmxPort->availableForWrite();
	pinMode(_pinEnable, OUTPUT);
	digitalWrite(_pinEnable, HIGH); // there is no need to switch it after, in fact it could just be hardwired
	pinMode(_pinToggle, OUTPUT);
	digitalWrite(_pinToggle, _ledOn);  // turns it on.
	_configured = true;
	return true;
}

bool dmxTx::transmit()
{
	if (!_configured) {
		return false;
	}
	_dmxPort->flush();	// flush() still causes it to be blocking, until all is transmitted
						// With ESP32 this function is overloaded for void & bool, Not setting 'false' results in true. 
						// void uartFlush(uart_t* uart) { uartFlushTxOnly(uart, true); } no default is given for the argument.
						// With ESP32 this function is overloaded
	                    // therefore if you don't want to waste time, do not call it before it has completed.
						// but we have to make sure it has all been sent before switching the Baud-rate.
    _dmxPort->updateBaudRate(DMX_BREAK_BAUD);
    _dmxPort->write(0);  // for the duration of the break, the processor will be held here.
    _dmxPort->flush();   // ESP32 UART has a method for sending a break, but it does not include a MAB
    _dmxPort->updateBaudRate(DMX_BAUD);
	//_lastTransmit = micros();
    _dmxPort->write(_dmxBuf, DMX_CHANNELS);	
	toggleLed();
	return true;
}

const uint32_t dmxTx::transmitMicros() // returns the number of microSeconds required to send a whole frame
{
	return (DMX_CHANNELS * 11 * 4);  // eg bits per byte 8N2 = 11, us per Bit 250kbps = 4
}

const uint32_t dmxTx::breakLength() // returns the length of the break
{
	return (11 * 1000000 / DMX_BREAK_BAUD);
}

bool dmxTx::readyToTransmit() { // Just does it by means of calculation.
	/*if (micros() - _lastTransmit > transmitMicros()) {
		return true;
	}*/
	if (_writeSize > _dmxPort->availableForWrite()) {
		return false;
	}
	return true;
}

bool dmxTx::write(uint8_t data, uint16_t channel)  // modify a single channel
{
	if ((channel < 1) || (channel > 512)) {
		return false;
	}
	_dmxBuf[channel] = data;
	return true;
}

uint16_t dmxTx::writeBytes(uint8_t* data, uint16_t numBytes, uint16_t startChannel) // modify a few channels
{
	if ((startChannel >= DMX_CHANNELS) || (!startChannel) || (!numBytes)) {
		return 0;
	}
    if (numBytes + startChannel > DMX_CHANNELS) {
		numBytes = DMX_CHANNELS - startChannel;
	}
    uint8_t* startPtr = _dmxBuf + startChannel;
    memcpy(startPtr, data, numBytes);
	return numBytes;
}

uint8_t* dmxTx::dmxBuffer() // returns a pointer to the tx-buffer for direct access
{
	return _dmxBuf;
}

#if ESP_IDF_VERSION_MAJOR >= 5

inline void dmxTx::toggleLed() 
{  // called from an IRAM_ATTR function
	static bool ledOn = false;
	ledOn = !ledOn;
	if (ledOn) digitalWrite(_pinToggle, HIGH);
	else digitalWrite(_pinToggle, LOW);
}

#else

IRAM_ATTR inline void dmxTx::toggleLed() 
{  // called from an IRAM_ATTR function
	static bool ledOn = false;
	ledOn = !ledOn;
	if (ledOn) digitalWrite(_pinToggle, HIGH);
	else digitalWrite(_pinToggle, LOW);
}

#endif
