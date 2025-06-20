#pragma once

#ifdef __cplusplus
extern "C" {
#endif
// Names of the two supported ICs
// Used for reporting the correct voltage measurement (see getVoltage method)
enum gaugeType
{
	MAX17048 = 1,
	MAX17049 = 2
};


void max17048g_init(i2c_master_dev_handle_t dev_handle);
float max17048g_getSOC();
float max17048g_getVoltage();
uint8_t max17048g_getStatus();
uint8_t max17048g_sleep();

#ifdef __cplusplus
}	
#endif


