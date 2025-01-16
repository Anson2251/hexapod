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

typedef struct {
    double x;
    double y;
    double z;
} Coordination;

typedef struct {
    int a;
    int b;
    int c;
} HexapodLegServoDegree;

/**
 * Convert leg end point coordination to servo degrees
 */
HexapodLegServoDegree hexapod_leg_position_to_servo_degrees(Coordination coord);

void i2c_master_init();

#endif
