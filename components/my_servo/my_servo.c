#include <stdio.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "include/my_servo.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0    // I2C 端口号

void pca9685_init() {
    uint8_t mode1_data[2] = {PCA9685_MODE1, 0x20}; // 设置模式寄存器 1（启用自动递增）
    i2c_master_write_to_device(I2C_MASTER_NUM, PCA9685_ADDR, mode1_data, 2, pdMS_TO_TICKS(1000));

    // 设置 PWM 频率为 50Hz（适用于舵机）
    uint8_t prescale_data[2] = {PCA9685_PRESCALE, 121}; // 预分频值 = round(25MHz / (4096 * 50Hz)) - 1
    i2c_master_write_to_device(I2C_MASTER_NUM, PCA9685_ADDR, prescale_data, 2, pdMS_TO_TICKS(1000));

    // 重启 PCA9685
    uint8_t restart_data[2] = {PCA9685_MODE1, 0x80};
    i2c_master_write_to_device(I2C_MASTER_NUM, PCA9685_ADDR, restart_data, 2, pdMS_TO_TICKS(1000));
}

void set_servo_angle(uint8_t channel, uint16_t angle) {
    // 将角度转换为 PWM 占空比
    uint16_t pulse_width = (angle * 100) / 180 + 100; // MG90 的脉宽范围为 1ms（0°）到 2ms（180°）
    uint16_t on_count = 0;
    uint16_t off_count = pulse_width * 4096 / 20000; // 4096 是 12 位分辨率，20000 是 20ms 周期

    // 写入 PWM 值到 PCA9685
    uint8_t pwm_data[5] = {
        PCA9685_LED0_ON_L + 4 * channel, // LED 起始地址
        on_count & 0xFF,                 // ON_L
        (on_count >> 8) & 0x0F,          // ON_H
        off_count & 0xFF,                // OFF_L
        (off_count >> 8) & 0x0F          // OFF_H
    };
    i2c_master_write_to_device(I2C_MASTER_NUM, PCA9685_ADDR, pwm_data, 5, pdMS_TO_TICKS(1000));
}