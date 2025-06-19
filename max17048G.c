#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "max17048g.h"

#define I2C_TIMEOUT -1 // ms
#define UV_PER_LSB 0.000078125
#define PERCENT_PER_LSB 0.00390625 // 1/256

// Generic error:
// Wire.endTransmission will return:
// 0:success
// 1:data too long to fit in transmit buffer
// 2:received NACK on transmit of address
// 3:received NACK on transmit of data
// 4:other error
// So, let's use "5" as a generic error value
#define MAX17043_GENERIC_ERROR 5

///////////////////////////////////
// MAX1704x Register Definitions //
///////////////////////////////////
// All registers contain two bytes of data and span two addresses.
// Registers which are present on the MAX17048/49 only are prefixed with MAX17048_
#define MAX17043_VCELL 0x02     // R - 12-bit A/D measurement of battery voltage
#define MAX17043_SOC 0x04       // R - 16-bit state of charge (SOC)
#define MAX17043_MODE 0x06      // W - Sends special commands to IC
#define MAX17043_VERSION 0x08   // R - Returns IC version
#define MAX17048_HIBRT 0x0A     // R/W - (MAX17048/49) Thresholds for entering hibernate
#define MAX17043_CONFIG 0x0C    // R/W - Battery compensation (default 0x971C)
#define MAX17048_CVALRT 0x14    // R/W - (MAX17048/49) Configures vcell range to generate alerts (default 0x00FF)
#define MAX17048_CRATE 0x16     // R - (MAX17048/49) Charge rate 0.208%/hr
#define MAX17048_VRESET_ID 0x18 // R/W - (MAX17048/49) Reset voltage and ID (default 0x96__)
#define MAX17048_STATUS 0x1A    // R/W - (MAX17048/49) Status of ID (default 0x01__)
#define MAX17043_CMD 0xFE   // W - Sends special comands to IC

/// MAX1704X commands
#define MAX1704X_CMD_RESET 0x5400

///////////////////////////////////
// MAX17043 Config Register Bits //
///////////////////////////////////
#define MAX17043_CONFIG_SLEEP (1 << 7)
#define MAX17043_CONFIG_ALSC 0x0040 // MAX17048/49 only
#define MAX17043_CONFIG_ALERT (1 << 5)
#define MAX17043_CONFIG_THRESHOLD 0

/////////////////////////////////////
// MAX17043 Mode Register Commands //
/////////////////////////////////////
#define MAX17043_MODE_QUICKSTART 0x4000 // On the MAX17048/49 this also clears the EnSleep bit

/////////////////////////////////
// MAX17048 Mode Register Bits //
/////////////////////////////////
#define MAX17048_MODE_ENSLEEP 0x2000 // W - _Enables_ sleep mode (the SLEEP bit in the CONFIG reg engages sleep)
#define MAX17048_MODE_HIBSTAT 0x1000 // R - indicates when the IC is in hibernate mode

/////////////////////////////////////
// MAX17048/9 Status Register Bits //
/////////////////////////////////////
#define MAX1704x_STATUS_RI (1 << 0) // Assumes the MSB has been shifted >> 8
#define MAX1704x_STATUS_VH (1 << 1) // Assumes the MSB has been shifted >> 8
#define MAX1704x_STATUS_VL (1 << 2) // Assumes the MSB has been shifted >> 8
#define MAX1704x_STATUS_VR (1 << 3) // Assumes the MSB has been shifted >> 8
#define MAX1704x_STATUS_HD (1 << 4) // Assumes the MSB has been shifted >> 8
#define MAX1704x_STATUS_SC (1 << 5) // Assumes the MSB has been shifted >> 8
#define MAX1704x_STATUS_EnVR (1 << 14) // ** Unshifted **

////////////////////////////////////////
// MAX17043 Command Register Commands //
////////////////////////////////////////
#define MAX17043_CMD_POR 0x5400

///////////////////////////////////////
// MAX17048 Hibernate Enable/Disable //
///////////////////////////////////////
#define MAX17048_HIBRT_ENHIB 0xFFFF // always use hibernate mode
#define MAX17048_HIBRT_DISHIB 0x0000 // disable hibernate mode

#define CONSTRAIN(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

static i2c_master_dev_handle_t dev_handle_MAX17048G;
static const char TAG[] = "MAX17048G";

static int32_t i2c_write(uint8_t reg, const uint8_t* bufp, uint16_t len)
{
	uint8_t data_wr[4 + 1];
	data_wr[0] = reg;
	for (int i = 0; i < len; i++)
		data_wr[i + 1] = bufp[i];
	esp_err_t err = i2c_master_transmit(dev_handle_MAX17048G, data_wr, len + 1, I2C_TIMEOUT);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "i2c_write error: %s", esp_err_to_name(err));
		return -1;
	}
	return 0;
}

static int32_t i2c_read(uint8_t reg, uint8_t* bufp, uint16_t len)
{
	esp_err_t err = i2c_master_transmit_receive(dev_handle_MAX17048G, &reg, 1, bufp, len, I2C_TIMEOUT);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "i2c_read error: %s", esp_err_to_name(err));
		return -1;
	}
	return 0;
}
static uint16_t read16(uint8_t reg)
{
	uint8_t data[2] = { 0,0 };
	i2c_read(reg, data, 2);
	return data[0] << 8 | data[1];
}

static uint8_t write16(uint8_t reg, uint16_t data)
{
	uint8_t data_wr[2] = { data >> 8, data & 0xFF };
	return i2c_write(reg, data_wr, 2);
}

static uint8_t clearStatusRegBits(uint16_t mask)
{
	uint16_t statusReg = read16(MAX17048_STATUS);
	statusReg &= ~mask; // Clear the specified bits
	return (write16(MAX17048_STATUS, statusReg)); // Write the contents back again
}

static uint8_t setResetVoltage(uint8_t threshold)
{
	uint16_t vreset = read16(MAX17048_VRESET_ID);
	vreset &= 0x01FF;                     // Mask out bits to set
	vreset |= ((uint16_t)threshold << 9); // Add new threshold

	return write16(MAX17048_VRESET_ID, vreset);
}

// VALRT Register:
//  This register is divided into two thresholds: Voltage alert
//  maximum (VALRT.MAX) and minimum (VALRT. MIN).
//  Both registers have 1 LSb = 20mV. The IC alerts while
//  VCELL > VALRT.MAX or VCELL < VALRT.MIN
static uint8_t setVALRTMax(uint8_t threshold)
{
	uint16_t valrt = read16(MAX17048_CVALRT);
	valrt &= 0xFF00; // Mask off max bits
	valrt |= (uint16_t)threshold;
	return write16(valrt, MAX17048_CVALRT);
}

// It forces the MAX1704X to restart fuel-gauge calculations
static uint8_t max17048g_quickStart()
{
	// A quick-start allows the MAX17043 to restart fuel-gauge calculations in the
	// same manner as initial power-up of the IC. If an application’s power-up
	// sequence is exceedingly noisy such that excess error is introduced into the
	// IC’s “first guess” of SOC, the host can issue a quick-start to reduce the
	// error. A quick-start is initiated by a rising edge on the QSTRT pin, or
	// through software by writing 4000h to MODE register.
	// Note: on the MAX17048/49 this will also clear / disable EnSleep

	return write16(MAX17043_MODE, MAX17043_MODE_QUICKSTART);
}

// It forces the MAX1704X to completely reset
static uint8_t max17048g_reset()
{
	return write16(MAX17043_CMD, MAX17043_CMD_POR);
}

// Returns the production version of the IC
static uint16_t max17048g_getVersion()
{
	return read16(MAX17043_VERSION);
}

/// @brief Returns the voltage of the connected LiIon Polymer battery
/// @return The voltage of the connected LiIon Polymer battery
float max17048g_getVoltage()
{
	return read16(MAX17043_VCELL) * UV_PER_LSB;
}

/// @brief Returns the relative state of charge of the connected 
/// LiIon Polymer battery as a percentage of the full capacity with
/// resolution 1/256%
float max17048g_getSOC()
{
	float p = read16(MAX17043_SOC) * PERCENT_PER_LSB;
	if (p > 100.0)
		p = 100.0;	
	return p;
}

// Return the LSByte of the CONFIG register
uint8_t max17048g_getStatus()
{
	uint8_t statusReg = read16(MAX17048_STATUS) >> 8;
	return (statusReg & 0x7F); //Highest bit is don't care
}

// Return the alert threshold as a percentage, below an alert interrupt is generated
uint8_t max17048g_getAlertThreshold()
{
	return (max17048g_getStatus() & 0x1F) + 1;
}

uint8_t max17048g_setThreshold(uint8_t percent)
{
	// The alert threshold is a 5-bit value that sets the state of charge level
	// where an interrupt is generated on the ALRT pin.

	// It has an LSb weight of 1%, and can be programmed from 1% to 32%.
	// The value is (32 - ATHD)%, e.g.: 00000=32%, 00001=31%, 11111=1%.
	// Let's convert our percent to that first:
	percent = (uint8_t)CONSTRAIN((float)percent, 0.0, 32.0);
	percent = 32 - percent;

	// Read config reg, so we don't modify any other values:
	uint16_t configReg = read16(MAX17043_CONFIG);
	configReg &= 0xFFE0;  // Mask out threshold bits
	configReg |= percent; // Add new threshold

	return write16(MAX17043_CONFIG, configReg);
}

uint8_t max17048g_getThreshold()
{
	uint16_t configReg = read16(MAX17043_CONFIG);
	uint8_t threshold = (configReg & 0x001F);

	// It has an LSb weight of 1%, and can be programmed from 1% to 32%.
	// The value is (32 - ATHD)%, e.g.: 00000=32%, 00001=31%, 11111=1%.
	// Let's convert our percent to that first:
	threshold = 32 - threshold;
	return threshold;
}

// enableSOCAlert() - (MAX17048/49) Enable the SOC change alert
// Returns true if the SOC change alert was enabled successfully
bool max17048g_enableSOCAlert()
{
	// Read config reg, so we don't modify any other values:
	uint16_t configReg = read16(MAX17043_CONFIG);
	configReg |= MAX17043_CONFIG_ALSC; // Set the ALSC bit
	// Update the config register, return false if the write fails
	if (write16(MAX17043_CONFIG, configReg) > 0)
		return (false);
	// Re-Read the config reg
	configReg = read16(MAX17043_CONFIG);
	// Return true if the ALSC bit is set, otherwise return false
	return ((configReg & MAX17043_CONFIG_ALSC) > 0);
}

// disableSOCAlert() - (MAX17048/49) Disable the SOC change alert
// Returns true if the SOC change alert was disabled successfully
bool max17048g_disableSOCAlert()
{
	// Read config reg, so we don't modify any other values:
	uint16_t configReg = read16(MAX17043_CONFIG);
	configReg &= ~MAX17043_CONFIG_ALSC; // Clear the ALSC bit
	// Update the config register, return false if the write fails
	if (write16(MAX17043_CONFIG, configReg) > 0)
		return (false);
	// Re-Read the config reg
	configReg = read16(MAX17043_CONFIG);
	// Return true if the ALSC bit is clear, otherwise return false
	return ((configReg & MAX17043_CONFIG_ALSC) == 0);
}

uint8_t max17048g_wake()
{
	// Read config reg, so we don't modify any other values:
	uint16_t configReg = read16(MAX17043_CONFIG);
	if (!(configReg & MAX17043_CONFIG_SLEEP))
		return MAX17043_GENERIC_ERROR; // Already sleeping, do nothing but return an error

	configReg &= ~MAX17043_CONFIG_SLEEP; // Clear sleep bit

	uint8_t result = write16(MAX17043_CONFIG, configReg);

	if (result)
		return (result); // Write failed. Bail.

	// On the MAX17048, we should also clear the EnSleep bit in the MODE register
	// Strictly, this will clear the QuickStart bit too. Which is probably a good thing,
	// as I don't think we can do a read-modify-write?
	return write16(MAX17043_MODE, 0x0000);
}

uint8_t max17048g_setVALRTMax(float threshold)
{
	uint8_t thresh = (uint8_t)(CONSTRAIN(threshold, 0.0, 5.1) / 0.02);
	return setVALRTMax(thresh);
}

uint8_t max17048g_getVALRTMax()
{
	uint16_t valrt = read16(MAX17048_CVALRT);
	valrt &= 0x00FF; // Mask off max bits
	return ((uint8_t)valrt);
}

static uint8_t setVALRTMin(uint8_t threshold)
{
	uint16_t valrt = read16(MAX17048_CVALRT);
	valrt &= 0x00FF; // Mask off min bits
	valrt |= ((uint16_t)threshold) << 8;
	return write16(valrt, MAX17048_CVALRT);
}

uint8_t max17048g_setVALRTMin(float threshold)
{
	uint8_t thresh = (uint8_t)(CONSTRAIN(threshold, 0.0, 5.1) / 0.02);
	return setVALRTMin(thresh);
}

uint8_t max17048g_getVALRTMin()
{
	uint16_t valrt = read16(MAX17048_CVALRT);
	valrt >>= 8; // Shift min into LSB
	return ((uint8_t)valrt);
}

uint8_t max17048g_getCompensation()
{
	uint16_t configReg = read16(MAX17043_CONFIG);
	uint8_t compensation = (configReg & 0xFF00) >> 8;
	return compensation;
}

uint16_t max17048g_getConfigRegister()
{
	return read16(MAX17043_CONFIG);
}

uint8_t max17048g_setCompensation(uint8_t newCompensation)
{
	// The CONFIG register compensates the ModelGauge algorith. The upper 8 bits
	// of the 16-bit register control the compensation.
	// Read the original configReg, so we can leave the lower 8 bits alone:
	uint16_t configReg = read16(MAX17043_CONFIG);
	configReg &= 0x00FF; // Mask out compensation bits
	configReg |= ((uint16_t)newCompensation) << 8;
	return write16(configReg, MAX17043_CONFIG);
}

uint8_t max17048g_setResetVoltage(float threshold)
{
	uint8_t thresh = (uint8_t)(CONSTRAIN(threshold, 0.0, 5.08) / 0.04);
	return setResetVoltage(thresh);
}

uint8_t max17048g_enableComparator(void)
{
	uint16_t vresetReg = read16(MAX17048_VRESET_ID);
	vresetReg &= ~(1 << 8); //Clear bit to enable comparator
	return write16(MAX17048_VRESET_ID, vresetReg);
}

// In sleep mode, the IC halts all operations, reducing current
// consumption to below 1μA. After exiting sleep mode,
// the IC continues normal operation. In sleep mode, the
// IC does not detect self-discharge. If the battery changes
// state while the IC sleeps, the IC cannot detect it, causing
// SOC error. Wake up the IC before charging or discharging.
uint8_t max17048g_sleep()
{
	// On the MAX17048, we also have to set the EnSleep bit in the MODE register
	uint8_t result = write16(MAX17048_MODE_ENSLEEP, MAX17043_MODE);
	if (result)
		return (result); // Write failed. Bail.

	// Read config reg, so we don't modify any other values:
	uint16_t configReg = read16(MAX17043_CONFIG);
	if (configReg & MAX17043_CONFIG_SLEEP)
		return MAX17043_GENERIC_ERROR; // Already sleeping, do nothing but return an error

	configReg |= MAX17043_CONFIG_SLEEP; // Set sleep bit

	return write16(configReg, MAX17043_CONFIG);
}

uint8_t max17048g_disableComparator(void)
{
	uint16_t vresetReg = read16(MAX17048_VRESET_ID);
	vresetReg |= (1 << 8); //Set bit to disable comparator
	return write16(MAX17048_VRESET_ID, vresetReg);
}

bool max17048g_isReset(bool clear)
{
	uint8_t status = max17048g_getStatus();
	bool flag = (status & MAX1704x_STATUS_RI) > 0;
	if (flag && clear) // Clear the flag if requested
		clearStatusRegBits(MAX1704x_STATUS_RI << 8);
	return (flag);
}
bool max17048g_isVoltageHigh(bool clear)
{
	uint8_t status = max17048g_getStatus();
	bool flag = (status & MAX1704x_STATUS_VH) > 0;
	if (flag && clear) // Clear the flag if requested
		clearStatusRegBits(MAX1704x_STATUS_VH << 8);
	return (flag);
}
bool max17048g_isVoltageLow(bool clear)
{
	uint8_t status = max17048g_getStatus();
	bool flag = (status & MAX1704x_STATUS_VL) > 0;
	if (flag && clear) // Clear the flag if requested
		clearStatusRegBits(MAX1704x_STATUS_VL << 8);
	return (flag);
}
bool max17048g_isVoltageReset(bool clear)
{
	uint8_t status = max17048g_getStatus();
	bool flag = (status & MAX1704x_STATUS_VR) > 0;
	if (flag && clear) // Clear the flag if requested
		clearStatusRegBits(MAX1704x_STATUS_VR << 8);
	return (flag);
}
bool max17048g_isLow(bool clear)
{
	uint8_t status = max17048g_getStatus();
	bool flag = (status & MAX1704x_STATUS_HD) > 0;
	if (flag && clear) // Clear the flag if requested
		clearStatusRegBits(MAX1704x_STATUS_HD << 8);
	return (flag);
}
bool max17048g_isChange(bool clear)
{
	uint8_t status = max17048g_getStatus();
	bool flag = (status & MAX1704x_STATUS_SC) > 0;
	if (flag && clear) // Clear the flag if requested
		clearStatusRegBits(MAX1704x_STATUS_SC << 8);
	return (flag);
}
void max17048g_init(i2c_master_dev_handle_t dev_handle)
{
	dev_handle_MAX17048G = dev_handle;

	max17048g_quickStart();
	max17048g_isReset(true);
	max17048g_setResetVoltage(3.0);
	max17048g_setThreshold(10);
	max17048g_setCompensation(0x971C);
	max17048g_enableComparator();
	max17048g_enableSOCAlert();
}



	// printf("MAX17048G init, version %X\n", max17048g_getVersion());
	// printf("    Voltage: %.2fV\n", max17048g_getVoltage());
	// printf("    SOC: %.2f%%\n", max17048g_getSOC());
	// printf("    Alert Threshold: %d%%\n", max17048g_getAlertThreshold());
	// printf("    Comparator: %s\n", max17048g_isVoltageHigh(false) ? "High" : "Low");
	// printf("    Reset: %s\n", max17048g_isReset(false) ? "True" : "False");
	// printf("    Voltage High: %s\n", max17048g_isVoltageHigh(false) ? "True" : "False");
	// printf("    Voltage Low: %s\n", max17048g_isVoltageLow(false) ? "True" : "False");
	// printf("    Voltage Reset: %s\n", max17048g_isVoltageReset(false) ? "True" : "False");
	// printf("    SOC Low: %s\n", max17048g_isLow(false) ? "True" : "False");
	// printf("    SOC Change: %s\n", max17048g_isChange(false) ? "True" : "False");
	// printf("    Compensation: %d\n", max17048g_getCompensation());
	// printf("    VALRT Max: %d\n", max17048g_getVALRTMax());
	// printf("    VALRT Min: %d\n", max17048g_getVALRTMin());


	// uint8_t status = max17048g_getStatus();
	// printf("    Status: Device Reset %d, V High %d, V Low %d, V Reset %d, SOC Low %d, SOC 1%% %d\n",
	// 	status & 0x01, status & 0x02, status & 0x04, status & 0x08, status & 0x10, status & 0x20);
