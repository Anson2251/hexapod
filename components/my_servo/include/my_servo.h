#ifndef DriverServo_h
#define DriverServo_h

#include <stdint.h>

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define PCA9685_ADDR 0x40
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define SERVO_MIN 150
#define SERVO_MAX 600

#define SERVO_DEG_RANGE 180

void i2c_master_init();

void pca9685_init();

void pca9685_write_byte(uint8_t reg, uint8_t data);

void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);

void set_servo_angle(uint8_t channel, uint16_t angle);

#endif
