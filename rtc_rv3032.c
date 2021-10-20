#include "rtc_rv3032.h"
#include "gshell.h"
#include "gfc32vf103_twi.h"

#define RV3032_ADDRESS	0xA2

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

}

enum RV3032_ERROR rv3032_getTime(RV3032_t *rtc, RV3032_TIME_t *time)
{

}

float rv3032_getTemperature(RV3032_t *rtc)
{

}

void rv3032_configureClockOut(RV3032_t *rtc, enum RV3032_CLKOUT clockOut, uint16_t hfClock_steps, enum RV3032_EEPROM_OPTION eeOption)
{

}