#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "driver/i2c.h"

#define M_PI 3.14159265358979323846

#define I2C_MASTER_SCL_IO 22        // I2C 时钟引脚
#define I2C_MASTER_SDA_IO 21        // I2C 数据引脚
#define I2C_MASTER_FREQ_HZ 100000   // I2C 频率 (100kHz)
#define I2C_MASTER_NUM I2C_NUM_0    // I2C 端口号

#define HexapodLegL1 0.5
#define HexapodLegL2 0.5
#define HexapodLegL3 0.5
#define CoreHeight 0.3
#define LegRiseHeight 0.5
#define Wingspan 0.5
#define Step 0.3

#define ServoMaxDegree 180
#define ServoMinDegree 0
#define ServoCalibrationFactor 1

typedef struct {
    double x;
    double y;
    double z;
} Coordination;

typedef struct {
    uint8_t a;
    uint8_t b;
    uint8_t c;
} HexapodLegServoDegree;

uint8_t calibrate_degree(uint8_t degree);

/**
 * Convert leg end point coordination to servo degrees
 */
HexapodLegServoDegree hexapod_leg_position_to_servo_degrees(Coordination coord);

void set_leg_angle(uint8_t leg_id, HexapodLegServoDegree degrees);

void hexapod_init();

#endif
