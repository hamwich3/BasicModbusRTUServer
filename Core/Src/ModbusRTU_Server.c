/*
 * ModbusRTU_Server.c
 *
 *  Created on: Jul 28, 2020
 *      Author: David Lutz
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "ModbusRTU_Server.h"


uint8_t coil[NUM_COILS] = {0};
uint8_t input[NUM_INPUTS] = {0};
uint16_t holdingRegister[NUM_HOLDINGREGISTERS] = {0};
uint16_t inputRegister[NUM_INPUTREGISTERS] = {0};

uint8_t MODBUS_Address = 1;

/* Table of CRC values for high–order byte */
static unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

/* Table of CRC values for low–order byte */
static char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};

void MODBUS_Init(){
	for (int i = 0; i < NUM_COILS; i++){
		coil[i] = 0;
	};
}

void MODBUS_SetAddress(uint8_t address){
	//Valid is range 0 - 247
	if (address >= 0 && address <= 247)
		MODBUS_Address = address;
}

uint16_t MODBUS_ProcessQuery(uint8_t *readBuffer, uint8_t messageLength,
		uint8_t *responseBuffer) {
	//ReceiveSerial(readBuffer);

	uint8_t address = readBuffer[0];
	uint8_t function = readBuffer[1];
	if (address != MODBUS_Address) {
		//flush serial buffer
		return 0;
	}

	CRC_ERROR = (!CheckCRC(readBuffer, messageLength));

	//MODBUS FUNCTIONS
	switch (function) {
	case READ_COIL_STATUS:
		return BuildReadCoilStatusResponse(readBuffer[2], readBuffer[3],
				readBuffer[4], readBuffer[5], responseBuffer);
		break;

	case READ_INPUT_STATUS:
		return BuildReadInputStatusResponse(readBuffer[2], readBuffer[3],
				readBuffer[4], readBuffer[5], responseBuffer);
		break;

	case READ_HOLDING_REGISTERS:
		return BuildReadHoldingRegistersResponse(readBuffer[2], readBuffer[3],
				readBuffer[4], readBuffer[5], responseBuffer);
		break;

	case READ_INPUT_REGISTERS:
		return BuildReadInputRegistersResponse(readBuffer[2], readBuffer[3],
				readBuffer[4], readBuffer[5], responseBuffer);
		break;

	case FORCE_SINGLE_COIL:
		return BuildForceSingleCoilResponse(readBuffer[2], readBuffer[3],
				readBuffer[4], readBuffer[5], responseBuffer);
		break;

	case PRESET_SINGLE_REGISTER:
		return BuildPresetSingleRegisterResponse(readBuffer[2], readBuffer[3],
				readBuffer[4], readBuffer[5], responseBuffer);
		break;

	case FORCE_MULTIPLE_COILS:
		return BuildForceMultipleCoilsResponse(readBuffer[2], readBuffer[3],
				readBuffer[4], readBuffer[5], readBuffer[6],
				&readBuffer[7], responseBuffer);
		break;

	case PRESET_MULTIPLE_REGISTERS:
		return BuildPresetMultipleRegistersResponse(readBuffer[2], readBuffer[3],
				readBuffer[4], readBuffer[5], readBuffer[6],
				&readBuffer[7], responseBuffer);
		break;

	case REPORT_SLAVE_ID:
		return BuildReportSlaveIDResponse(responseBuffer);
		break;

	default:
		break;
	}
	return 0;
}

_Bool CheckCRC(uint8_t *data, uint8_t messageLength) {
	uint16_t crc = CalculateCRC(data, messageLength - 2);
	return (data[messageLength - 2] == (char) (crc & 0x00FF)
			&& data[messageLength - 1] == (char) ((crc & 0xFF00) >> 8));
}

uint16_t CalculateCRC(uint8_t *data, uint8_t length) {
	uint16_t crc = 0xFFFF;
	for (int byte = 0; byte < length; byte++) {
		crc ^= data[byte];
		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x1) {
				crc >>= 1;
				crc ^= 0xA001;
			} else
				crc >>= 1;
		}
	}
	return crc;
}

uint16_t CRC16(unsigned char * puchMsg, unsigned short usDataLen){
	//unsigned char *puchMsg ; /* message to calculate CRC upon */
	//unsigned short usDataLen ; /* quantity of bytes in message */

	unsigned char uchCRCHi = 0xFF ; /* high byte of CRC initialized */
	unsigned char uchCRCLo = 0xFF ; /* low byte of CRC initialized */
	uint8_t uIndex ; /* will index into CRC lookup table */

	while (usDataLen--){
		uIndex = uchCRCHi ^ *puchMsg++;
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	return (uchCRCLo << 8 | uchCRCHi);
}

uint8_t BuildExceptionResponse() {
	return 0;
}

//Function 01
uint8_t BuildReadCoilStatusResponse(uint8_t startingAddressHi,
		uint8_t startingAddressLo, uint8_t noOfPointsHi, uint8_t noOfPointsLo,
		uint8_t *responseBuffer) {

	if (startingAddressHi >= 0x40){
		startingAddressHi -= 0x40;
	}
	uint16_t address = (startingAddressHi << 8) + startingAddressLo;
	uint16_t points = (noOfPointsHi << 8) + noOfPointsLo;
	uint8_t byteCount = (points / 8) + ((points % 8) == 0 ? 0 : 1);

	uint8_t index = 0;

	responseBuffer[index++] = MODBUS_Address;
	responseBuffer[index++] = READ_COIL_STATUS;
	responseBuffer[index++] = byteCount;

	//load coil states
	for (int i = 0; i < points; i++) {
		//zero out byte or old data will remain
		if (i % 8 == 0) responseBuffer[index + (i / 8)] = 0;
		responseBuffer[index + (i / 8)] |= (int) coil[address + i] << i % 8;
	}

	index += byteCount;

	//Append CRC
	uint16_t crc = CRC16(responseBuffer, index);
	responseBuffer[index++] = (char) (crc & 0x00FF);
	responseBuffer[index++] = (char) ((crc & 0xFF00) >> 8);
	return index;
}

//Function 02
uint8_t BuildReadInputStatusResponse(uint8_t startingAddressHi,
		uint8_t startingAddressLo, uint8_t noOfPointsHi, uint8_t noOfPointsLo,
		uint8_t *responseBuffer) {

	uint16_t address = (startingAddressHi << 8) + startingAddressLo;
	uint16_t points = (noOfPointsHi << 8) + noOfPointsLo;
	uint8_t byteCount = (points / 8) + ((points % 8) == 0 ? 0 : 1);

	uint8_t index = 0;

	responseBuffer[index++] = MODBUS_Address;
	responseBuffer[index++] = READ_INPUT_STATUS;
	responseBuffer[index++] = byteCount;


	//load input states
	for (int i = 0; i < points; i++) {
		//zero out byte or old data will remain
		if (i % 8 == 0) responseBuffer[index + (i / 8)] = 0;
		responseBuffer[index + (i / 8)] |= (int) input[address + i] << i % 8;
	}

	index += byteCount;

	//Append CRC
	uint16_t crc = CRC16(responseBuffer, index);
	responseBuffer[index++] = (char) (crc & 0x00FF);
	responseBuffer[index++] = (char) ((crc & 0xFF00) >> 8);
	return index;
}

//Function 03
uint8_t BuildReadHoldingRegistersResponse(uint8_t startingAddressHi,
		uint8_t startingAddressLo, uint8_t noOfRegistersHi,
		uint8_t noOfRegistersLo, uint8_t *responseBuffer) {

	uint16_t address = (startingAddressHi << 8) + startingAddressLo;
	uint16_t registerCount = (noOfRegistersHi << 8) + noOfRegistersLo;

	uint8_t byteCount = registerCount * 2;

	uint8_t index = 0;

	responseBuffer[index++] = MODBUS_Address;
	responseBuffer[index++] = READ_HOLDING_REGISTERS;
	responseBuffer[index++] = byteCount;

	for (int i = 0; i < registerCount; i++) {
		responseBuffer[index++] = (int) holdingRegister[address + i] >> 8;
		responseBuffer[index++] = (int) holdingRegister[address + i] & 0xFF;
	}

	//Append CRC
	uint16_t crc = CRC16(responseBuffer, index);
	responseBuffer[index++] = (char) (crc & 0x00FF);
	responseBuffer[index++] = (char) ((crc & 0xFF00) >> 8);
	return index;
}

//Function 04
uint8_t BuildReadInputRegistersResponse(uint8_t startingAddressHi,
		uint8_t startingAddressLo, uint8_t noOfRegistersHi,
		uint8_t noOfRegistersLo, uint8_t *responseBuffer) {

	uint16_t address = (startingAddressHi << 8) + startingAddressLo;
	uint16_t registerCount = (noOfRegistersHi << 8) + noOfRegistersLo;

	uint8_t byteCount = registerCount * 2;

	uint8_t index = 0;

	responseBuffer[index++] = MODBUS_Address;
	responseBuffer[index++] = READ_INPUT_REGISTERS;
	responseBuffer[index++] = byteCount;

	for (int i = 0; i < registerCount; i++) {
		responseBuffer[index++] = (int) inputRegister[address + i] >> 8;
		responseBuffer[index++] = (int) inputRegister[address + i] & 0xFF;
	}

	//Append CRC
	uint16_t crc = CRC16(responseBuffer, index);
	responseBuffer[index++] = (char) (crc & 0x00FF);
	responseBuffer[index++] = (char) ((crc & 0xFF00) >> 8);
	return index;
}

//Function 05
uint8_t BuildForceSingleCoilResponse(uint8_t coilAddressHi, uint8_t coilAddressLo,
		uint8_t forceDataHi, uint8_t forceDataLo, uint8_t *responseBuffer){

	uint16_t address = (coilAddressHi << 8) + coilAddressLo;
	if (address >= 0x4000){
		address -= 0x4000;
	}

	uint16_t forceData = (forceDataHi << 8) + forceDataLo;

	if (forceData == 0xFF00) {
		coil[address] = 1;
	}
	else if (forceData == 0x0000) {
			coil[address] = 0;
	}
	else{
		//error state
	}

	uint8_t index = 0;

	responseBuffer[index++] = MODBUS_Address;
	responseBuffer[index++] = FORCE_SINGLE_COIL;
	responseBuffer[index++] = coilAddressHi;
	responseBuffer[index++] = coilAddressLo;
	responseBuffer[index++] = forceDataHi;
	responseBuffer[index++] = forceDataLo;

	//Append CRC
	uint16_t crc = CRC16(responseBuffer, index);
	responseBuffer[index++] = (char) (crc & 0x00FF);
	responseBuffer[index++] = (char) ((crc & 0xFF00) >> 8);
	return index;
}

//Function 06
uint8_t BuildPresetSingleRegisterResponse(uint8_t registerAddressHi, uint8_t registerAddressLo,
		uint8_t presetDataHi, uint8_t presetDataLo, uint8_t *responseBuffer){

	uint16_t address = (registerAddressHi << 8) + registerAddressLo;
	uint16_t presetData = (presetDataHi << 8) + presetDataLo;

	holdingRegister[address] = presetData;

	uint8_t index = 0;

	responseBuffer[index++] = MODBUS_Address;
	responseBuffer[index++] = PRESET_SINGLE_REGISTER;
	responseBuffer[index++] = registerAddressHi;
	responseBuffer[index++] = registerAddressLo;
	responseBuffer[index++] = presetDataHi;
	responseBuffer[index++] = presetDataLo;

	//Append CRC
	uint16_t crc = CRC16(responseBuffer, index);
	responseBuffer[index++] = (char) (crc & 0x00FF);
	responseBuffer[index++] = (char) ((crc & 0xFF00) >> 8);
	return index;
}

//Function 15
uint8_t BuildForceMultipleCoilsResponse(uint8_t coilAddressHi, uint8_t coilAddressLo,
		uint8_t quantityOfCoilsHi, uint8_t quantityOfCoilsLo, uint8_t byteCount,
		uint8_t *forceData, uint8_t *responseBuffer){

	uint16_t address = (coilAddressHi << 8) + coilAddressLo;
	uint16_t quantity = (quantityOfCoilsHi << 8) + quantityOfCoilsLo;

	for (uint8_t i = 0; i < quantity; i++){

		coil[address + i] = (forceData[i / 8] & (0x1 << (i % 8))) > 0;
	}

	uint8_t index = 0;
	responseBuffer[index++] = MODBUS_Address;
	responseBuffer[index++] = FORCE_MULTIPLE_COILS;
	responseBuffer[index++] = coilAddressHi;
	responseBuffer[index++] = coilAddressLo;
	responseBuffer[index++] = quantityOfCoilsHi;
	responseBuffer[index++] = quantityOfCoilsLo;

	//Append CRC
	uint16_t crc = CRC16(responseBuffer, index);
	responseBuffer[index++] = (char) (crc & 0x00FF);
	responseBuffer[index++] = (char) ((crc & 0xFF00) >> 8);
	return index;
}

//Function 16
uint8_t BuildPresetMultipleRegistersResponse(uint8_t startingAddressHi, uint8_t startingAddressLo,
		uint8_t noOfRegistersHi, uint8_t noOfRegistersLo, uint8_t byteCount,
		uint8_t *data, uint8_t *responseBuffer){

	uint16_t address = (startingAddressHi << 8) + startingAddressLo;
	uint16_t quantity = (noOfRegistersHi << 8) + noOfRegistersLo;

	for (uint8_t i = 0; i < quantity; i++){
		holdingRegister[address + i] = (data[2 * i] << 8) + data[(2 * i) + 1];
	}

	uint8_t index = 0;
	responseBuffer[index++] = MODBUS_Address;
	responseBuffer[index++] = PRESET_MULTIPLE_REGISTERS;
	responseBuffer[index++] = startingAddressHi;
	responseBuffer[index++] = startingAddressLo;
	responseBuffer[index++] = noOfRegistersHi;
	responseBuffer[index++] = noOfRegistersLo;

	//Append CRC
	uint16_t crc = CRC16(responseBuffer, index);
	responseBuffer[index++] = (char) (crc & 0x00FF);
	responseBuffer[index++] = (char) ((crc & 0xFF00) >> 8);
	return index;
}

//Function 17
uint8_t BuildReportSlaveIDResponse(uint8_t *responseBuffer){

	uint8_t index = 0;
	responseBuffer[index++] = MODBUS_Address;
	responseBuffer[index++] = REPORT_SLAVE_ID;
	responseBuffer[index++] = 22;
	responseBuffer[index++] = 0xFF;

	//Append CRC
	uint16_t crc = CRC16(responseBuffer, index);
	responseBuffer[index++] = (char) (crc & 0x00FF);
	responseBuffer[index++] = (char) ((crc & 0xFF00) >> 8);
	return index;
}

#ifdef __cplusplus
}
#endif
