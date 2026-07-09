#include <math.h>

#include <zephyr/logging/log.h>

#include "LIS2MDL.h"
#include "LIS3MDL.h" // Common functions

static const float sensitivity = 1.5 / 1000; // ~1.5 mgauss/LSB -> 0.0015 G/LSB

static uint8_t last_odr = 0xff;

LOG_MODULE_REGISTER(LIS2MDL, LOG_LEVEL_DBG);

int lis2_init(float time, float *actual_time)
{
	// CFG_REG_C: BDU always set; 4WSPI + I2C_DIS only in SPI mode
	// (I2C mode must not set I2C_DIS or it cuts off the bus)
	enum sensor_interface_spec spec = sensor_interface_get_spec(SENSOR_INTERFACE_DEV_MAG);
	uint8_t cfg_c = LIS2MDL_BDU;
	if (spec == SENSOR_INTERFACE_SPEC_SPI)
		cfg_c |= LIS2MDL_4WSPI | LIS2MDL_I2C_DIS;
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_C, cfg_c);

	// CFG_REG_B: LPF (ODR/4 bandwidth) + OFF_CANC (internal bias cancellation)
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_B, LIS2MDL_LPF | LIS2MDL_OFF_CANC);
	if (err)
		LOG_ERR("Communication error");

	last_odr = 0xff; // reset last odr
	err = lis2_update_odr(time, actual_time);
	return (err < 0 ? err : 0);
}

void lis2_shutdown(void)
{
	last_odr = 0xff; // reset last odr
	// MD_IDLE instead of SOFT_RST (0x20) to preserve CFG_REG_B/C settings
	// (SOFT_RST resets all config registers, losing 4WSPI/BDU/LPF/OFF_CANC)
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_A, COMP_TEMP_EN | MD_IDLE);
	if (err)
		LOG_ERR("Communication error");
}

int lis2_update_odr(float time, float *actual_time)
{
	int ODR;
	uint8_t MODR;
	uint8_t MD;

	if (time <= 0) // off
	{
		MD = MD_IDLE;
		ODR = 0;
	}
	else if (time == INFINITY) // oneshot/single
	{
//		MD = MD_SINGLE;
//		ODR = 0;
		MD = MD_IDLE; // No idea if single measurements will be fast enough, so just use continuous anyway
		ODR = 0;
	}
	else
	{
		MD = MD_CONTINUOUS;
		ODR = 1 / time;
	}

	if (MD == MD_IDLE)
	{
		MODR = 0;
		time = 0; // off
	}
	else if (ODR > 50) // TODO: this sucks
	{
		MODR = ODR_100Hz;
		time = 1.0 / 100;
	}
	else if (ODR > 20)
	{
		MODR = ODR_50Hz;
		time = 1.0 / 50;
	}
	else if (ODR > 10)
	{
		MODR = ODR_20Hz;
		time = 1.0 / 20;
	}
	else if (ODR > 0)
	{
		MODR = ODR_10Hz;
		time = 1.0 / 10;
	}
	else
	{
		MODR = 0;
		time = INFINITY;
	}

	if (last_odr == MODR)
		return 1;
	else
		last_odr = MODR;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_A, COMP_TEMP_EN | MODR << 2 | MD); // set mag ODR and MD (temp comp must be on)
	if (err)
		LOG_ERR("Communication error");

	*actual_time = time;
	return err;
}

void lis2_mag_oneshot(void)
{
	// write MD_SINGLE again to trigger a measurement
	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_CFG_REG_A, COMP_TEMP_EN | last_odr << 2 | MD_SINGLE); // set mag ODR and MD (temp comp must be on)
	if (err)
		LOG_ERR("Communication error");
	// Single mode auto-returns to idle after measurement; reset last_odr so the
	// next update_odr() re-writes the mode instead of skipping (last_odr == MODR)
	// and leaving the device stuck in idle.
	last_odr = 0xff;
}

bool lis2_mag_read(float m[3])
{
	// Read all 6 data registers every call. This always releases the BDU lock
	// (BDU=1 + a skipped/partial data read would otherwise freeze the output
	// regs and stall data-ready forever) and removes the old uninitialized
	// `status` while-loop that could spin on a transient SPI error and hang the
	// sensor loop. Always return true so mag_sample advances every loop pass;
	// duplicate samples are filtered downstream (fusion time-gate, calibration
	// direction-diversity gate), so flow control here is unnecessary and a
	// data-ready gate has proven fragile with this part's BDU/continuous combo.
	uint8_t rawData[6];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_OUTX_L_REG, &rawData[0], 6);
	if (err)
	{
		LOG_ERR("Communication error");
		return false;
	}
	lis2_mag_process(rawData, m);
	return true;
}

float lis2_temp_read(float bias[3])
{
	uint8_t rawTemp[2];
	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_MAG, LIS2MDL_TEMP_OUT_L_REG, &rawTemp[0], 2);
	// The output value is expressed as a signed 16-bit byte in two’s complement.
	// The four most significant bits contain a copy of the sign bit.
	// The nominal sensitivity is 8 LSB/°C
	float temp = (int16_t)((((uint16_t)rawTemp[1]) << 8) | rawTemp[0]);
	temp /= 8;
	// No value offset?
	if (err)
		LOG_ERR("Communication error");
	return temp;
}

void lis2_mag_process(uint8_t *raw_m, float m[3])
{
	for (int i = 0; i < 3; i++) // x, y, z
	{
		m[i] = (int16_t)((((uint16_t)raw_m[(i * 2) + 1]) << 8) | raw_m[i * 2]);
		m[i] *= sensitivity;
	}
}

const sensor_mag_t sensor_mag_lis2mdl = {
	*lis2_init,
	*lis2_shutdown,

	*lis2_update_odr,

	*lis2_mag_oneshot,
	*lis2_mag_read,
	*lis2_temp_read,

	*lis2_mag_process,
	6, 6,
	1.5f
};
