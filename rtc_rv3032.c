#include "rtc_rv3032.h"
#include "gshell.h"
#include "gfc32vf103_twi.h"

#define RV3032_ADDRESS	0xA2

static uint8_t bcd2bin(uint8_t val)
{
	return val - 6 * ((val & 0xF0) >> 4);
}

static uint8_t bin2bcd(uint8_t val)
{
	return ((val / 10) << 4) | (val % 10);
}

void rv3032_initStruct(RV3032_t *rtc, void *ioInterface, uint8_t (*startTrans)(void*), uint8_t (*sendBytes)(void*,uint8_t,uint8_t*,uint16_t),uint8_t (*getBytes)(void*,uint8_t,uint8_t*,uint16_t),uint8_t (*endTrans)(void*), void (*wait1ms)(void))
{
	rtc->ioInterface = ioInterface;
	rtc->startTransaction = startTrans;
	rtc->sendBytes = sendBytes;
	rtc->getBytes = getBytes;
	rtc->endTransaction = endTrans;
	rtc->wait1ms = wait1ms;
	rtc->errors = RV3032_NO_ERRORS;
}

uint8_t rv3032_init(RV3032_t *rtc)
{
	/* No fancy ID registers make things a little bit hard - lets just see if we can read
	 * the controller without errors */
	rv3032_readReg(rtc, R_RV3032_100TH_SECONDS);
	
	return rtc->errors;
}

void rv3032_writeReg(RV3032_t *rtc, uint8_t reg_addr, uint8_t reg_val)
{
	uint8_t dataPackage[2] = {reg_addr, reg_val};

	rtc->startTransaction(rtc->ioInterface);
	rtc->errors |= rtc->sendBytes(rtc->ioInterface, RV3032_ADDRESS, dataPackage, 2);
	rtc->endTransaction(rtc->ioInterface);
}

uint8_t rv3032_readReg(RV3032_t *rtc, uint8_t reg_addr)
{
	uint8_t dataPackage = reg_addr;

	rtc->startTransaction(rtc->ioInterface);

	/* Point to the internal register... */
	rtc->errors |= rtc->sendBytes(rtc->ioInterface, RV3032_ADDRESS, &dataPackage, 1);
	/* ... and read the value back */
	rtc->errors |= rtc->getBytes(rtc->ioInterface, RV3032_ADDRESS, &dataPackage, 1);
	
	rtc->endTransaction(rtc->ioInterface);

	return dataPackage;
}

enum RV3032_ERROR rv3032_setTime(RV3032_t *rtc, RV3032_TIME_t *time)
{
	uint8_t dataPackage[9];
	
	dataPackage[0] = R_RV3032_100TH_SECONDS;
	dataPackage[1] = bin2bcd(time->hSeconds);
	dataPackage[2] = bin2bcd(time->seconds);
	dataPackage[3] = bin2bcd(time->minutes);
	dataPackage[4] = bin2bcd(time->hours);
	dataPackage[5] = bin2bcd(time->weekday);
	dataPackage[6] = bin2bcd(time->date);
	dataPackage[7] = bin2bcd(time->month);
	dataPackage[8] = bin2bcd(time->year);

	rtc->errors = RV3032_NO_ERRORS;
	rtc->startTransaction(rtc->ioInterface);
	rtc->errors |= rtc->sendBytes(rtc->ioInterface, RV3032_ADDRESS, dataPackage, 9);
	rtc->endTransaction(rtc->ioInterface);

	return rtc->errors;
}

enum RV3032_ERROR rv3032_getTime(RV3032_t *rtc, RV3032_TIME_t *time)
{
	uint8_t dataPackage[8];

	dataPackage[0] = R_RV3032_100TH_SECONDS;

	rtc->errors = RV3032_NO_ERRORS;

	rtc->startTransaction(rtc->ioInterface);

	/* Point to the internal register... */
	rtc->errors |= rtc->sendBytes(rtc->ioInterface, RV3032_ADDRESS, dataPackage, 1);
	/* ... and read the value back */
	rtc->errors |= rtc->getBytes(rtc->ioInterface, RV3032_ADDRESS, dataPackage, 8);
	
	rtc->endTransaction(rtc->ioInterface);

	time->hSeconds 	= bcd2bin(dataPackage[0]);
	time->seconds 	= bcd2bin(dataPackage[1]);
	time->minutes 	= bcd2bin(dataPackage[2]);
	time->hours 	= bcd2bin(dataPackage[3]);
	time->weekday 	= bcd2bin(dataPackage[4]);
	time->date 		= bcd2bin(dataPackage[5]);
	time->month 	= bcd2bin(dataPackage[6]);
	time->year 		= bcd2bin(dataPackage[7]);

	return rtc->errors;
}

float rv3032_getTemperature(RV3032_t *rtc)
{
	uint8_t dataPackage[2];
	int16_t calcVar = 0;
	float temp_celsius;

	rtc->errors = RV3032_NO_ERRORS;

	dataPackage[0] = R_RV3032_TEMPERATURE_L;

	rtc->startTransaction(rtc->ioInterface);

	rtc->errors |= rtc->sendBytes(rtc->ioInterface, RV3032_ADDRESS, dataPackage, 1);
	rtc->errors |= rtc->getBytes(rtc->ioInterface, RV3032_ADDRESS, dataPackage, 2);

	rtc->endTransaction(rtc->ioInterface);

	if (rtc->errors != RV3032_NO_ERRORS)	return 255.0f;

	calcVar = ((dataPackage[1] & 0x7F) << 4) | ((dataPackage[0] & 0xF0) >> 4);

	if (dataPackage[1] & (1<<7))	calcVar |= (1<<15);

	temp_celsius = (float)calcVar * 0.0625;

	return temp_celsius;
}

uint32_t rv3032_getTemperatureUINT(RV3032_t *rtc)
{
	uint8_t dataPackage[2];
	int16_t calcVar = 0;
	uint32_t temp_celsius;

	rtc->errors = RV3032_NO_ERRORS;

	dataPackage[0] = R_RV3032_TEMPERATURE_L;

	rtc->startTransaction(rtc->ioInterface);

	rtc->errors |= rtc->sendBytes(rtc->ioInterface, RV3032_ADDRESS, dataPackage, 1);
	rtc->errors |= rtc->getBytes(rtc->ioInterface, RV3032_ADDRESS, dataPackage, 2);

	rtc->endTransaction(rtc->ioInterface);

	if (rtc->errors != RV3032_NO_ERRORS)	return 255.0f;

	calcVar = ((dataPackage[1] & 0x7F) << 4) | ((dataPackage[0] & 0xF0) >> 4);

	if (dataPackage[1] & (1<<7))	calcVar |= (1<<15);

	temp_celsius = (uint32_t)calcVar * 625;

	return temp_celsius;
}

void rv3032_writeEEPROM(RV3032_t *rtc, uint8_t eeprom_addr, uint8_t eeprom_val)
{
	uint8_t autoRefreshReg;
	rtc->errors = RV3032_NO_ERRORS;

	/* Check if the eeprom is busy first */
	rv3032_writeReg(rtc, R_RV3032_TEMPERATURE_L, 0);
	while(rv3032_readReg(rtc, R_RV3032_TEMPERATURE_L) & R_RV3032_TEMPERATURE_L_EEBUSY)
	{
		rtc->wait1ms();
	}

	/* Disable eeprom auto read, if enabled */
	autoRefreshReg = rv3032_readReg(rtc, R_RV3032_CONTROL_1);
	if (autoRefreshReg & R_RV3032_CONTROL_1_EERD)
	{
		rv3032_writeReg(rtc, R_RV3032_CONTROL_1, autoRefreshReg & ~R_RV3032_CONTROL_1_EERD);
	}


	rv3032_writeReg(rtc, R_RV3032_EE_ADDRESS, eeprom_addr);
	rv3032_writeReg(rtc, R_RV3032_EE_DATA, eeprom_val);
	rv3032_writeReg(rtc, R_RV3032_EE_COMMAND, R_RV3032_EE_COMMAND_WRITE);

	/* Re-enable eeprom auto read, if it was previousely enabled */
	if (autoRefreshReg & R_RV3032_CONTROL_1_EERD)
	{
		rv3032_writeReg(rtc, R_RV3032_CONTROL_1, autoRefreshReg);
	}
}

uint8_t rv3032_readEEPROM(RV3032_t *rtc, uint8_t eeprom_addr)
{
	uint8_t eeprom_val, autoRefreshReg;
	rtc->errors = RV3032_NO_ERRORS;

	/* Check if the eeprom is busy first */
	rv3032_writeReg(rtc, R_RV3032_TEMPERATURE_L, 0);
	while(rv3032_readReg(rtc, R_RV3032_TEMPERATURE_L) & R_RV3032_TEMPERATURE_L_EEBUSY)
	{
		rtc->wait1ms();
	}

	/* Disable eeprom auto read, if enabled */
	autoRefreshReg = rv3032_readReg(rtc, R_RV3032_CONTROL_1);
	if (autoRefreshReg & R_RV3032_CONTROL_1_EERD)
	{
		rv3032_writeReg(rtc, R_RV3032_CONTROL_1, autoRefreshReg & ~R_RV3032_CONTROL_1_EERD);
	}

	rv3032_writeReg(rtc, R_RV3032_EE_ADDRESS, eeprom_addr);
	rv3032_writeReg(rtc, R_RV3032_EE_COMMAND, R_RV3032_EE_COMMAND_READ);

	rv3032_writeReg(rtc, R_RV3032_TEMPERATURE_L, 0);
	while(rv3032_readReg(rtc, R_RV3032_TEMPERATURE_L) & R_RV3032_TEMPERATURE_L_EEBUSY)
	{
		rtc->wait1ms();
	}

	eeprom_val = rv3032_readReg(rtc, R_RV3032_EE_DATA);

	/* Re-enable eeprom auto read, if it was previousely enabled */
	if (autoRefreshReg & R_RV3032_CONTROL_1_EERD)
	{
		rv3032_writeReg(rtc, R_RV3032_CONTROL_1, autoRefreshReg);
	}

	return eeprom_val;
}

enum RV3032_ERROR rv3032_configureClockOut(RV3032_t *rtc, enum RV3032_CLKOUT clockOut, uint16_t hfClock_steps)
{
	enum RV3032_ERROR _error = RV3032_NO_ERRORS;
	uint8_t regVal;
	rtc->errors = RV3032_NO_ERRORS;
	
	regVal = rv3032_readReg(rtc, E_RV3032_PMU) & ~E_RV3032_PMU_NCLKE;
	if (clockOut != RV3032_DISABLE_CLKOUT)
	{
		if (clockOut == RV3032_HF_MODE)
		{
			rv3032_writeEEPROM(rtc, E_RV3032_CLKOUT1, hfClock_steps);
			_error |= rtc->errors;
			rv3032_writeReg(rtc, E_RV3032_CLKOUT1, hfClock_steps);
			rv3032_writeEEPROM(rtc, E_RV3032_CLKOUT2, RV3032_HF_MODE | ((hfClock_steps >> 8) & 0x1F));
			_error |= rtc->errors;
			rv3032_writeReg(rtc, E_RV3032_CLKOUT2, RV3032_HF_MODE | ((hfClock_steps >> 8) & 0x1F));
		}
		else
		{
			rv3032_writeEEPROM(rtc, E_RV3032_CLKOUT2, clockOut);
			_error |= rtc->errors;
			rv3032_writeReg(rtc, E_RV3032_CLKOUT2, clockOut);
		}

		/* Enable ClockOut */
		regVal |= E_RV3032_PMU_NCLKE;
	}
	rv3032_writeEEPROM(rtc, E_RV3032_PMU, regVal);
	_error |= rtc->errors;
	rv3032_writeReg(rtc, E_RV3032_PMU, regVal);

	rtc->errors |= _error;
	return _error;	
}

enum RV3032_ERROR rv3032_configureBSM(RV3032_t *rtc, enum RV3032_BSM bsm)
{
	enum RV3032_ERROR _error;
	uint8_t regVal;

	regVal = rv3032_readReg(rtc, E_RV3032_PMU) & ~(E_RV3032_PMU_BSM_1 | E_RV3032_PMU_BSM_1);

	_error = rtc->errors;
	regVal |= bsm;

	rv3032_writeEEPROM(rtc, E_RV3032_PMU, regVal);
	rv3032_writeReg(rtc, E_RV3032_PMU, regVal);

	rtc->errors |= _error;
	return rtc->errors;
}

enum RV3032_ERROR rv3032_configureTrickleCharge(RV3032_t *rtc, enum RV3032_TCR tcr, enum RV3032_TCM tcm)
{
	enum RV3032_ERROR _error;
	uint8_t regVal;

	regVal = rv3032_readReg(rtc, E_RV3032_PMU) & (E_RV3032_PMU_NCLKE | E_RV3032_PMU_BSM_1 | E_RV3032_PMU_BSM_1);

	_error = rtc->errors;
	regVal |= tcr | tcm;

	rv3032_writeEEPROM(rtc, E_RV3032_PMU, regVal);
	rv3032_writeReg(rtc, E_RV3032_PMU, regVal);

	rtc->errors |= _error;
	return rtc->errors;
}