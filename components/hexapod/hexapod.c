#include <math.h>
#include "include/hexapod.h"
#include "driver/i2c.h"

double atan_deg(double x){
    return atan(x) * 180 / M_PI;
}

HexapodLegServoDegree hexapod_leg_position_to_servo_degrees(Coordination coord) {
    HexapodLegServoDegree degrees;
    long square_sum = pow(coord.x, 2) + pow(coord.y, 2) + pow(coord.z, 2);
    degrees.a = atan_deg(coord.y / coord.x);
    degrees.b = 
        atan_deg(
            -pow(HexapodLegL3, 2) + pow(HexapodLegL2, 2) + square_sum) 
            / (2 * HexapodLegL2 * sqrt(square_sum)
        ) + 
        atan_deg(coord.z / sqrt(pow(coord.x, 2) + pow(coord.y, 2)));

    degrees.c = - atan_deg((square_sum - pow(HexapodLegL2, 2) - pow(HexapodLegL3, 2)) / (2 * HexapodLegL2 * HexapodLegL3));

    return degrees;
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
