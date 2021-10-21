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

void rv3032_initStruct(RV3032_t *rtc, void *ioInterface, uint8_t (*startTrans)(void*), uint8_t (*sendBytes)(void*,uint8_t,uint8_t*,uint16_t),uint8_t (*getBytes)(void*,uint8_t,uint8_t*,uint16_t),uint8_t (*endTrans)(void*))
{
	rtc->ioInterface = ioInterface;
	rtc->startTransaction = startTrans;
	rtc->sendBytes = sendBytes;
	rtc->getBytes = getBytes;
	rtc->endTransaction = endTrans;
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
	rtc->errors = RV3032_NO_ERRORS;

	rtc->startTransaction(rtc->ioInterface);
	rtc->errors = rtc->sendBytes(rtc->ioInterface, RV3032_ADDRESS, dataPackage, 2);
	rtc->endTransaction(rtc->ioInterface);
}

extern TWI_Master_t twim;
uint8_t rv3032_readReg(RV3032_t *rtc, uint8_t reg_addr)
{
	uint8_t dataPackage = reg_addr;

	rtc->errors = RV3032_NO_ERRORS;

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

void rv3032_configureClockOut(RV3032_t *rtc, enum RV3032_CLKOUT clockOut, uint16_t hfClock_steps, enum RV3032_EEPROM_OPTION eeOption)
{
	uint8_t dataPackage;

	rtc->errors = RV3032_NO_ERRORS;

	switch(clockOut)
	{
		case RV3032_DISABLE_CLKOUT:

			break;
		case RV3032_XTAL_32KHZ_OUT:

			break;
		case RV3032_XTAL_1024HZ_OUT:

			break;
		case RV3032_XTAL_64HZ_OUT:

			break;
		case RV3032_XTAL_1HZ_OUT:

			break;
		case RV3032_HF_MODE:

			break;
		default:

			break;
	}
}