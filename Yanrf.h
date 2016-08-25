/*
MIT License

Copyright (c) 2016 Roman Chomik romanchom@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef NRF24L01_h
#define NRF24L01_h

#include <cstdint>
#include <SPI.h>

class Yanrf {
private:
	uint8_t mPinCE;
	uint8_t mPinCSN;
	uint8_t mAddressLength;
	uint8_t mConfigReg;
	SPISettings mSpiSettings;

	void writeRegister(uint8_t reg, uint8_t value);
	uint8_t readRegister(uint8_t reg);

	enum commands {
		CMD_READ_REGISTER = 0x00,
		CMD_WRITE_REGISTER = 0x20,
		CMD_READ_RECEIVE_PAYLOAD = 0x61,
		CMD_WRITE_TRANSMIT_PAYLOAD = 0xA0,
		CMD_FLUSH_TRANSMIT = 0xE1,
		CMD_FLUSH_RECEIVE = 0xE2,
		CMD_REUSE_TRANSMIT_PAYLOAD = 0xE3,
		CMD_READ_RECEIVE_PAYLOAD_LENGTH = 0x60,
		CMD_WRITE_ACK_PAYLOAD = 0xA8,
		CMD_WRITE_TRANSMIT_PAYLOAD_NOACK = 0xB0,
		CMD_NOP = 0xFF,
	};
	enum registers {
		REG_CONFIG = 0x00,
		REG_ENABLE_AUTO_ACK,
		REG_ENABLE_RECEIVE_PIPES,
		REG_SET_ADDRESS_LENGTH,
		REG_SET_RETRANSMISSION,
		REG_SET_RF_CHANNEL,
		REG_SET_RF_RATE_POWER,
		REG_STATUS,
		REG_OBSERVE_TRANSMIT_STATS,
		REG_RECEIVED_POWER_DETECTOR,
		REG_RECEIVER_ADDRESS_PIPE0,
		REG_RECEIVER_ADDRESS_PIPE1,
		REG_RECEIVER_ADDRESS_PIPE2,
		REG_RECEIVER_ADDRESS_PIPE3,
		REG_RECEIVER_ADDRESS_PIPE4,
		REG_RECEIVER_ADDRESS_PIPE5,
		REG_TRANSMITTER_ADDRESS,
		REG_RECEIVE_PAYLOAD_LENGTH_PIPE0,
		REG_RECEIVE_PAYLOAD_LENGTH_PIPE1,
		REG_RECEIVE_PAYLOAD_LENGTH_PIPE2,
		REG_RECEIVE_PAYLOAD_LENGTH_PIPE3,
		REG_RECEIVE_PAYLOAD_LENGTH_PIPE4,
		REG_RECEIVE_PAYLOAD_LENGTH_PIPE5,
		REG_FIFO_STATUS,
		REG_DYNAMIC_PAYLOAD = 0x1C,
		REG_FEATURE = 0x1D,
	};
	enum bitValues {
		// REG_CONFIG
		ENABLE_CRC = 0x08,
		CRC_ENCODING = 0x04, // 0 - 8 bits, 1 - 16 bits
		POWER_UP = 0x02,
		RX_MODE = 0x01, // 0 - primary transmitter, 1 - primary receiver

		// REG_SET_RF_RATE_POWER

		// REG_STATUS
		RECEIVE_DATA_READY = 0x40,
		TRANSMIT_SUCCESSFUL = 0x20,
		TRANSMIT_FAIL = 0x10,
		RECEIVE_PIPE_NUMBER = 0x0E,
		TRANSMIT_FULL = 0x01,

		// REG_FIFO_STATUS
		TRANSMIT_REUSE = 0x40,
		TRANSMIT_FULL_FIFO = 0x20,
		TRANSMIT_EMPTY = 0x10,
		RECEIVE_FULL = 0x02,
		RECEIVE_EMPTY = 0x01,

		// REG_FEATURE
		ENABLE_DYNAMIC_PAYLOAD = 0x04,
		ENABLE_ACK_PAYLOAD = 0x02,
		ENABLE_DYNAMIC_ACK = 0x01,
	};
public:
	Yanrf();

	void setCE(uint8_t value){
		mPinCE = value;
	}
	void setCSN(uint8_t value){
		mPinCSN = value;
	}
	void setSpiClock(uint32_t clock){
		mSpiSettings = SPISettings(clock, MSBFIRST, SPI_MODE0);
	}

	void init(uint32_t powerOnResetDelay = 100000);
	//void setPower(bool powerOn, uint32_t crystalStartupDelay = 1500);

	// length must be between 3 and 5
	void setAdressLength(uint8_t length);
	// same TX and RX address
	void setNetworkAdress(const uint8_t * address);
	void setReceivePipe(uint8_t pipe, const uint8_t * address);

	inline void setMode(uint8_t mode){
		mConfigReg &= ~RX_MODE;
		if(mode == MODE_RECEIVER) mConfigReg |= RX_MODE;
		writeRegister(REG_CONFIG, mConfigReg);
	}
	inline void setRetransmissions(uint8_t timeOut, uint8_t count){
		writeRegister(REG_SET_RETRANSMISSION, (timeOut << 4) | count);
	}

	inline void setDataRateAndPower(uint8_t flags){
		writeRegister(REG_SET_RF_RATE_POWER, flags);
	}

	inline void setChannel(uint8_t channel){
		writeRegister(REG_SET_RF_CHANNEL, channel);
	}

	void sendPacket(const void * data, uint8_t count);
	uint8_t getTransmissionState();
	uint8_t waitForEndOfTransmission(uint32_t timeout = 1000);
	void setListening(bool enable){
		digitalWriteFast(mPinCE, enable);
	}

	uint8_t available();
	void read(void * dst, uint8_t count);



	enum returnValues{
		SUCCESSFUL = 0x01,
		PENDING = 0x02,
		FAIL = 0x04,
		UNDEFINED = 0x08, // indicates there is a problem with radio, possibly connection error
		TIMEOUT = 0x10,
		UNSUCCESSFUL = FAIL | UNDEFINED | TIMEOUT,
	};
	enum {
		MODE_TRANSMITTER = 0x00,
		MODE_RECEIVER = 0x01,

		RF_DATA_RATE_250K = 0x20,
		RF_DATA_RATE_1M = 0x00,
		RF_DATA_RATE_2M = 0x08,
		RF_POWER__18DBM = 0x00,
		RF_POWER__12DBM = 0x02,
		RF_POWER__6DBM = 0x04,
		RF_POWER_0DBM = 0x06,
	};
private:
	inline uint8_t beginCommand(uint8_t cmd){
		SPI.beginTransaction(mSpiSettings);
		digitalWriteFast(mPinCSN, 0);
		return SPI.transfer(cmd);
	}

	inline void endCommand(){
		digitalWriteFast(mPinCSN, 1);
		SPI.endTransaction();
	}
};






#endif // NRF24L01_h
