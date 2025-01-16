#ifndef DriverServo_h
#define DriverServo_h

#include <stdint.h>

#define PCA9685_ADDR 0x40           // PCA9685 的 I2C 地址
#define PCA9685_MODE1 0x00          // 模式寄存器 1
#define PCA9685_PRESCALE 0xFE       // 预分频寄存器
#define PCA9685_LED0_ON_L 0x06      // LED0 起始地址

void i2c_master_init();

/**
 * @brief Initialize the PCA9685 servo driver
 */
void pca9685_init();

/**
 * @brief Set the angle of a servo channel
 */
void set_servo_angle(uint8_t channel, uint16_t angle);

#endif