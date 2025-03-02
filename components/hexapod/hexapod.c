#include <math.h>
#include "include/hexapod.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "my_servo.h"

#define HEXAPOD_TAG "HEXAPOD"

/**
 * @brief Convert radians to degrees.
 * @param x The angle in radians to be converted.
 * @return The angle in degrees.
 */
double atan_deg(double x)
{
	return atan(x) * 180 / M_PI;
}

/**
 * @brief Calibrate the degree of the servo.
 * @param degree The degree to be calibrated.
 * @return The calibrated degree.
 */
uint8_t calibrate_degree(uint8_t degree)
{
	if (degree > ServoMinDegree && degree < ServoMaxDegree)
	{
		return degree;
	}
	return (degree % (ServoMaxDegree - ServoMinDegree) + ServoMinDegree) * ServoCalibrationFactor;
}

/**
 * @brief Convert the coordination to servo degrees.
 * @param coord The coordination to be converted.
 * @return The servo degrees.
 */
HexapodLegServoDegree hexapod_leg_position_to_servo_degrees(Coordination coord)
{
	HexapodLegServoDegree degrees;
	long square_sum = pow(coord.x, 2) + pow(coord.y, 2) + pow(coord.z, 2);
	degrees.a = calibrate_degree((uint8_t)atan_deg(coord.y / coord.x));
	degrees.b =
		calibrate_degree((uint8_t)(atan_deg(
									   -pow(HexapodLegL3, 2) + pow(HexapodLegL2, 2) + square_sum) /
									   (2 * HexapodLegL2 * sqrt(square_sum)) +
								   atan_deg(coord.z / sqrt(pow(coord.x, 2) + pow(coord.y, 2)))));

	degrees.c = calibrate_degree((uint8_t)(-atan_deg((square_sum - pow(HexapodLegL2, 2) - pow(HexapodLegL3, 2)) / (2 * HexapodLegL2 * HexapodLegL3))));

	return degrees;
}

/**
 * Leg angle calibration
 * @param base_servo_id The base servo id (`uint8`, `<4-bit-channel><4-bit-leg-id>`)
 * @param degree Degree to calibrate
 * @return Calibrated degree
 */
void set_leg_angle(uint8_t base_servo_id, HexapodLegServoDegree degrees)
{
	uint8_t addr = ((base_servo_id & 0xF0) >> 4) + 0x40;
	base_servo_id = (base_servo_id  & 0x0F);
	ESP_LOGI(HEXAPOD_TAG, "Setting leg angle for channel 0x%x, base_servo %d", addr, base_servo_id);
	set_servo_angle(addr, base_servo_id + 0, degrees.a);
	set_servo_angle(addr, base_servo_id + 1, degrees.b);
	set_servo_angle(addr, base_servo_id + 2, degrees.c);
}

void hexapod_init()
{
	i2c_master_init(I2C_NUM_0);
	i2c_master_init(I2C_NUM_1);
	pca9685_init(0x40);
	pca9685_init(0x41);
	ESP_LOGI(HEXAPOD_TAG, "Hexapod system initialized");
}
