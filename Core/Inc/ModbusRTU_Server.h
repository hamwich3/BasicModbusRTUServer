/*
 * ModbusRTU_Slave.h
 *
 *  Created on: Jul 28, 2020
 *      Author: David Lutz
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define NUM_COILS 210
#define NUM_INPUTS 0
#define NUM_HOLDINGREGISTERS 12
#define NUM_INPUTREGISTERS 20

//Define Modbus Function Codes
#define READ_COIL_STATUS			01
#define READ_INPUT_STATUS			02
#define READ_HOLDING_REGISTERS		03
#define READ_INPUT_REGISTERS		04
#define FORCE_SINGLE_COIL			05
#define PRESET_SINGLE_REGISTER		06
#define FORCE_MULTIPLE_COILS		15
#define PRESET_MULTIPLE_REGISTERS	16
#define REPORT_SLAVE_ID				17

//Define Modbus Error Codes



_Bool CRC_ERROR;

uint8_t MODBUS_Address;

void MODBUS_Init();

void MODBUS_SetAddress(uint8_t address);

uint16_t MODBUS_ProcessQuery(uint8_t *readBuffer, uint8_t messageLength, uint8_t *responseBuffer);

//Calculate a 16-bit CRC from data bytes
//https://modbus.org/docs/PI_MBUS_300.pdf
uint16_t CalculateCRC(uint8_t *data, uint8_t length);
unsigned short CRC16(unsigned char * puchMsg, unsigned short usDataLen);

_Bool CheckCRC(uint8_t *data, uint8_t messageLength);

//Function 01
uint8_t BuildReadCoilStatusResponse(uint8_t startingAddressHi,
		uint8_t startingAddressLo, uint8_t noOfPointsHi, uint8_t noOfPointsLo, uint8_t *responseBuffer);

//Function 02
uint8_t BuildReadInputStatusResponse(uint8_t startingAddressHi,
		uint8_t startingAddressLo, uint8_t noOfPointsHi, uint8_t noOfPointsLo, uint8_t *responseBuffer);

//Function 03
uint8_t BuildReadHoldingRegistersResponse(uint8_t startingAddressHi,
		uint8_t startingAddressLo, uint8_t noOfRegistersHi,
		uint8_t noOfRegistersLo, uint8_t *responseBuffer);

//Function 04
uint8_t BuildReadInputRegistersResponse(uint8_t startingAddressHi,
		uint8_t startingAddressLo, uint8_t noOfRegistersHi,
		uint8_t noOfRegistersLo, uint8_t *responseBuffer);

//Function 05
uint8_t BuildForceSingleCoilResponse(uint8_t coilAddressHi, uint8_t coilAddressLo,
		uint8_t forceDataHi, uint8_t forceDataLo, uint8_t *responseBuffer);

//Function 06
uint8_t BuildPresetSingleRegisterResponse(uint8_t registerAddressHi, uint8_t registerAddressLo,
		uint8_t presetDataHi, uint8_t presetDataLo, uint8_t *responseBuffer);

//Function 15
uint8_t BuildForceMultipleCoilsResponse(uint8_t coilAddressHi, uint8_t coilAddressLo,
		uint8_t quantityOfCoilsHi, uint8_t quantityOfCoilsLo, uint8_t byteCount,
		uint8_t *forceData, uint8_t *responseBuffer);

//Function 16
uint8_t BuildPresetMultipleRegistersResponse(uint8_t startingAddressHi, uint8_t startingAddressLo,
		uint8_t noOfRegistersHi, uint8_t noOfRegistersLo, uint8_t byteCount,
		uint8_t *data, uint8_t *responseBuffer);

//Function 17
uint8_t BuildReportSlaveIDResponse(uint8_t *responseBuffer);

uint8_t BuildExceptionResponse();

#ifdef __cplusplus
}
#endif
