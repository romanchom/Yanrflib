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

#include "Yanrf.h"

Yanrf::Yanrf() :
	mPinCE(-1),
	mPinCSN(-1),
	mAddressLength(5),
	mSpiSettings(10000000, MSBFIRST, SPI_MODE0)
{}

void Yanrf::init(uint32_t powerOnResetDelay){
#ifndef NDEBUG
	bool OK = true;
	if(-1 == mPinCE){
		Serial.println("RadioRFN: Pin CE not set. Call setCE() before init().");
		OK = false;
	}
	if(-1 == mPinCSN){
		Serial.println("RadioRFN: Pin CSN not set. Call setCSN() before init().");
		OK = false;
	}
	if(!OK) return;
#endif
	pinMode(mPinCE, OUTPUT);
	pinMode(mPinCSN, OUTPUT);
	digitalWriteFast(mPinCE, 0);
	digitalWriteFast(mPinCSN, 1);

	delayMicroseconds(powerOnResetDelay);

	uint8_t status = beginCommand(CMD_NOP);
	endCommand();

	// simple test to see if we are talking
	// to something that remotely resembles nrf24l01
	if(status != RECEIVE_PIPE_NUMBER){
		Serial.println("RadioRFN: Unexpected status value. Is the connection good?");
	}
	mConfigReg = ENABLE_CRC | CRC_ENCODING | POWER_UP;
	writeRegister(REG_CONFIG, mConfigReg);
	uint32_t bootTime = micros();

	writeRegister(REG_DYNAMIC_PAYLOAD, 0x3F); // enable dynamic payload on all pipes

	writeRegister(REG_FEATURE,
		ENABLE_DYNAMIC_PAYLOAD | ENABLE_ACK_PAYLOAD | ENABLE_DYNAMIC_ACK);

	// flush all queues in case something is still theres
	flushReceive();
	flushTransmit();
	// wait till radio oscillator boots
	delayMicroseconds(micros() - bootTime);
}

void Yanrf::setAdressLength(uint8_t length){
	mAddressLength = length;
	beginCommand(CMD_WRITE_REGISTER | REG_SET_ADDRESS_LENGTH);
	SPI.transfer(length - 2);
	endCommand();
}

void Yanrf::setNetworkAdress(const uint8_t * address){
	beginCommand(CMD_WRITE_REGISTER | REG_TRANSMITTER_ADDRESS);
	for(int i = 0; i < mAddressLength; ++i){
		SPI.transfer(address[i]);
	}
	endCommand();
	setReceivePipe(0, address);
}

void Yanrf::setReceivePipe(uint8_t pipe, const uint8_t * address){
	int l = (pipe > 1) ? 1 : mAddressLength;
	beginCommand(CMD_WRITE_REGISTER | (REG_RECEIVER_ADDRESS_PIPE0 + pipe));
	for(int i = 0; i < l; ++i){
		SPI.transfer(address[i]);
	}
	endCommand();
}

void Yanrf::sendPacket(const void * data, uint8_t count){
	beginCommand(CMD_WRITE_TRANSMIT_PAYLOAD);
	for(const uint8_t * p = reinterpret_cast<const uint8_t *>(data), * end = p + count; p < end; ++p){
		SPI.transfer(*p);
	}
	endCommand();

	 // datasheet says keep CE pin high for more than 10us
	 // 11us is more than 10us
	digitalWriteFast(mPinCE, 1);
	delayMicroseconds(11);
	digitalWriteFast(mPinCE, 0);
}


uint8_t Yanrf::getTransmissionState(){
	uint16_t state = beginCommand(CMD_READ_REGISTER | REG_FIFO_STATUS);
	state |= (static_cast<uint16_t>(SPI.transfer(0)) << 8);
	endCommand();

	if(state & (TRANSMIT_FULL_FIFO << 8)) Serial.print("FIFO full");
	if(state & TRANSMIT_SUCCESSFUL) {
		if(state & RECEIVE_DATA_READY) return SUCCESSFUL | ACK_PAYLOAD_WAITING;
		else return SUCCESSFUL;
	}
	if(state & TRANSMIT_FAIL) {
		return FAIL;
	}

	if(0 == (state & (TRANSMIT_EMPTY << 8))) return PENDING;

	return UNDEFINED;
}

uint8_t Yanrf::waitForEndOfTransmission(uint32_t timeout){
	timeout += micros();
	uint8_t ret = TIMEOUT;
	do{
		uint8_t state = getTransmissionState();
		if(state & (FAIL | SUCCESSFUL)) {

			ret = state;
			break;
		}
		if(state & UNDEFINED) ret = UNDEFINED;
	}while(micros() < timeout);
	// clear all flags
	writeRegister(REG_STATUS, 0xFF);
	if(ret & UNSUCCESSFUL){
		 // if transmission fails we need to manualy flush
		 flushTransmit();
		 //flushReceive();
	}
	return ret;
}

uint8_t Yanrf::available(){
	uint8_t status = beginCommand(CMD_NOP);
	endCommand();

	uint8_t pipeNumber = (status >> 1) & 0x07;

	if(pipeNumber != 7){
		beginCommand(CMD_READ_RECEIVE_PAYLOAD_LENGTH);
		uint8_t count = SPI.transfer(0);
		endCommand();

		if(count > 32){ // something is mighty wrong
			beginCommand(CMD_FLUSH_RECEIVE);
			endCommand();
			return 0;
		}else{
			return count;
		}
	}else{
		return 0;
	}
}

void Yanrf::read(void * dst, uint8_t count){
	beginCommand(CMD_READ_RECEIVE_PAYLOAD);
	for(uint8_t * p = reinterpret_cast<uint8_t *>(dst), * end = p + count; p < end; ++p){
		*p = SPI.transfer(0);
	}
	endCommand();
}


void Yanrf::ackPayload(const void * data, uint8_t count, bool overwrite){
	if(overwrite) flushTransmit();

	beginCommand(CMD_WRITE_ACK_PAYLOAD);
	for(const uint8_t * p = reinterpret_cast<const uint8_t *>(data), * end = p + count; p < end; ++p){
		SPI.transfer(*p);
	}
	endCommand();
}

void Yanrf::flushReceive(){
	beginCommand(CMD_FLUSH_RECEIVE);
	endCommand();
}

void Yanrf::flushTransmit(){
	beginCommand(CMD_FLUSH_TRANSMIT);
	endCommand();
}


void Yanrf::writeRegister(uint8_t reg, uint8_t value){
	beginCommand(CMD_WRITE_REGISTER | reg);
	SPI.transfer(value);
	endCommand();
}
