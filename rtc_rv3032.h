
#ifndef _RTC_RV3032_H_
#define _RTC_RV3032_H_

#include <stdint.h>

/* Registers of the RV-3032-C7 */
#define R_RV3032_100TH_SECONDS	0x00
#define R_RV3032_SECONDS		0x01
#define R_RV3032_MINUTES		0x02
#define R_RV3032_HOURS			0x03
#define R_RV3032_WEEKDAY		0x04
#define R_RV3032_DATE			0x05
#define R_RV3032_MONTH			0x06
#define R_RV3032_YEAR			0x07
#define R_RV3032_MIN_ALARM		0x08
#define R_RV3032_HOUR_ALARM		0x09
#define R_RV3032_DATE_ALARM		0x0A
#define R_RV3032_TIMER_VALUE_0	0x0B
#define R_RV3032_TIMER_VALUE_1	0x0C
#define R_RV3032_STATUS			0x0D
#define R_RV3032_TEMPERATURE_L	0x0E
#define R_RV3032_TEMPERATURE_H	0x0F
#define R_RV3032_CONTROL_1		0x10
#define R_RV3032_CONTROL_2		0x11
#define R_RV3032_CONTROL_3		0x12
#define R_RV3032_TIMESTAMP_CTRL	0x13
#define R_RV3032_COCK_INT_MASK	0x14
#define R_RV3032_EVI_CONTROL	0x15
#define R_RV3032_TLOW_THRESHLD	0x16
#define R_RV3032_THIGH_THRESHLD	0x17
#define R_RV3032_TS_TLOW_COUNT	0x18
#define R_RV3032_TS_TLOW_SEC	0x19
#define R_RV3032_TS_TLOW_MIN	0x1A
#define R_RV3032_TS_TLOW_HOUR	0x1B
#define R_RV3032_TS_TLOW_DATE	0x1C
#define R_RV3032_TS_TLOW_MONTH	0x1D
#define R_RV3032_TS_TLOW_YEAR	0x1E
#define R_RV3032_TS_THIGH_COUNT	0x1F
#define R_RV3032_TS_THIGH_SEC	0x20
#define R_RV3032_TS_THIGH_MIN	0x21
#define R_RV3032_TS_THIGH_HOUR	0x22
#define R_RV3032_TS_THIGH_DATE	0x23
#define R_RV3032_TS_THIGH_MONTH	0x24
#define R_RV3032_TS_THIGH_YEAR	0x25
#define R_RV3032_TS_EVI_COUNT	0x26
#define R_RV3032_TS_EVI_100THS	0x27
#define R_RV3032_TS_EVI_SEC		0x28
#define R_RV3032_TS_EVI_MIN		0x29
#define R_RV3032_TS_EVI_HOUR	0x2A
#define R_RV3032_TS_EVI_DATE	0x2B
#define R_RV3032_TS_EVI_MONTH	0x2C
#define R_RV3032_TS_EVI_YEAR	0x2D
#define R_RV3032_PASSWORD_0		0x39
#define R_RV3032_PASSWORD_1		0x3A
#define R_RV3032_PASSWORD_2		0x3B
#define R_RV3032_PASSWORD_3		0x3C
#define R_RV3032_EE_ADDRESS		0x3D
#define R_RV3032_EE_DATA		0x3E
#define R_RV3032_EE_COMMAND		0x3F
#define R_RV3032_USER_RAM_START	0x40
#define R_RV3032_USER_RAM_END	0x4F

typedef struct {
	uint8_t HID:4;
	uint8_t VID:4;
	// errors
	void *ioInterface;					// InterfacePointer of the IO/Interface Library
	uint8_t (*startTransaction)(void*);	// Prepare the IO/Peripheral Interface for a transaction
	uint8_t (*sendBytes)(void*,			// Send data to the interface: InterfacePointer
						uint8_t,		// I2C Address of the PWM chip (8-Bit format!)
						uint8_t*,		// Pointer to the buffer to send
						uint16_t);		// length of the buffer
	uint8_t (*getBytes)(void*,			// Get data from the interface: InterfacePointer
						uint8_t,		// I2C Address of the PWM chip (8-Bit format!)
						uint8_t*,		// Pointer to the buffer to receive
						uint16_t);		// length of the buffer
	uint8_t (*endTransaction)(void*);	// Finish the transaction & release the IO / Peripheral
}RV3032_t;

/* _RTC_RV3032_H_ */
#endif