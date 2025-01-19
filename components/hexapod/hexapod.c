#include <math.h>
#include "include/hexapod.h"
#include "driver/i2c.h"
#include "my_servo.h"

/**
 * @brief Convert radians to degrees.
 * @param x The angle in radians to be converted.
 * @return The angle in degrees.
 */
double atan_deg(double x){
    return atan(x) * 180 / M_PI;
}

uint8_t calibrate_degree(uint8_t degree) {
    if (degree > ServoMinDegree && degree < ServoMaxDegree) {
        return degree;
    }
    return (degree % (ServoMaxDegree - ServoMinDegree) + ServoMinDegree) * ServoCalibrationFactor;
}

HexapodLegServoDegree hexapod_leg_position_to_servo_degrees(Coordination coord) {
    HexapodLegServoDegree degrees;
    long square_sum = pow(coord.x, 2) + pow(coord.y, 2) + pow(coord.z, 2);
    degrees.a = calibrate_degree((uint8_t)atan_deg(coord.y / coord.x));
    degrees.b = 
        calibrate_degree((uint8_t)(atan_deg(
            -pow(HexapodLegL3, 2) + pow(HexapodLegL2, 2) + square_sum) 
            / (2 * HexapodLegL2 * sqrt(square_sum)
        ) + 
        atan_deg(coord.z / sqrt(pow(coord.x, 2) + pow(coord.y, 2)))));

    degrees.c = calibrate_degree((uint8_t)(- atan_deg((square_sum - pow(HexapodLegL2, 2) - pow(HexapodLegL3, 2)) / (2 * HexapodLegL2 * HexapodLegL3))));

    return degrees;
}

void set_leg_angle(uint8_t leg_id, HexapodLegServoDegree degrees){
    set_servo_angle(leg_id * 3 + 0, degrees.a);
    set_servo_angle(leg_id * 3 + 1, degrees.b);
    set_servo_angle(leg_id * 3 + 2, degrees.c);
}

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}
