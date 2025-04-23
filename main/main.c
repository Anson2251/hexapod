#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_mac.h>
#include "my_servo.h"
#include "hexapod.h"
#include "esp_log.h"
#include <driver/ledc.h>

// 定义消息类型
typedef enum
{
    MSG_SENSOR_DATA,
    MSG_GAIT_COMMAND,
    MSG_SERVO_CONTROL
} message_type_t;

typedef enum
{
    GAIT_A,
    GAIT_B,
    GAIT_C,
    GAIT_D,
    GAIT_BACKWARD_A,
    GAIT_BACKWARD_B,
    GAIT_BACKWARD_C,
    GAIT_BACKWARD_D,
    GAIT_LEFT_A,
    GAIT_LEFT_B,
    GAIT_RIGHT_A,
    GAIT_RIGHT_B
} GAIT_TYPE;

// 定义消息结构体
typedef struct
{
    message_type_t type;
    union
    {
        struct
        {
            int sensor_value; // 传感器数据
        } sensor_data;
        struct
        {
            int gait_pattern; // 歩态模式
        } gait_command;
        struct
        {
            int8_t servo_id; // 舵机ID
            int8_t angle;    // 舵机角度
        } servo_control;
    };
} message_t;

typedef struct
{
    uint16_t leg_id;
    Coordination coord;
} ServoMessage;

// /** Sensor Data Queue */
// QueueHandle_t xSensorQueue;

/** Servo Command Queue */
QueueHandle_t xServoQueue;

// void sensor_task(void *pvParameters) {
//     while (1) {
//         // 模拟读取传感器数据
//         int sensor_value = 100; // 假设读取到的传感器数据

//         // 创建消息
//         message_t msg;
//         msg.type = MSG_SENSOR_DATA;
//         msg.sensor_data.sensor_value = sensor_value;

//         // 发送消息到传感器队列
//         if (xQueueSend(xSensorQueue, &msg, portMAX_DELAY) != pdPASS) {
//             printf("Failed to send sensor data to queue\n");
//         }

//         vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms读取一次传感器数据
//     }
// }

void set_leg_group(int8_t num, bool raise, bool forward, bool expand)
{
    for (int8_t i = 0; i < 3; i++)
    {
        uint8_t base_servo = (i * 3) | (num << 4); // <4-bit-addr><4-bit-base-servo-id>
        ServoMessage msg; // Use a stack-allocated variable
        msg.coord.x = Step ? forward : 0;
        msg.coord.y = expand ? Wingspan : ExpandWingspan;
        msg.coord.z = -(raise ? CoreHeight + LegRiseHeight : CoreHeight);
        msg.leg_id = base_servo; // one group has 3 legs, each leg has 3 servos

        if (xQueueSend(xServoQueue, &msg, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE("Gait Control", "Failed to send servo message for leg %x to queue\n", base_servo & 0x0F);
        }
        ESP_LOGI("Gait Control", "Servo message sent for leg %x: (X: %lf, Y: %lf, Z: %lf)", base_servo & 0x0F, msg.coord.x, msg.coord.y, msg.coord.z);
    }
}

void gait_control_task(void *pvParameters)
{
    int8_t gait_pattern = 0;
    while (1)
    {
        switch (gait_pattern)
        {
        // Forward movement gaits
        case GAIT_A:
            set_leg_group(0, false, false, false);
            set_leg_group(1, false, true, false);
            ESP_LOGI("Gait Control", "Gait A (Forward)");
            break;
        case GAIT_B:
            set_leg_group(0, true, true, false);
            set_leg_group(1, false, false, false);
            ESP_LOGI("Gait Control", "Gait B (Forward)");
            break;
        case GAIT_C:
            set_leg_group(0, false, true, false);
            set_leg_group(1, false, false, false);
            ESP_LOGI("Gait Control", "Gait C (Forward)");
            break;
        case GAIT_D:
            set_leg_group(0, false, false, false);
            set_leg_group(1, true, true, false);
            ESP_LOGI("Gait Control", "Gait D (Forward)");
            break;

        // Backward movement gaits
        case GAIT_BACKWARD_A:
            set_leg_group(0, false, false, false);
            set_leg_group(1, false, false, false);
            ESP_LOGI("Gait Control", "Gait BACKWARD A");
            break;
        case GAIT_BACKWARD_B:
            set_leg_group(0, true, false, false);
            set_leg_group(1, false, false, false);
            ESP_LOGI("Gait Control", "Gait BACKWARD B");
            break;
        case GAIT_BACKWARD_C:
            set_leg_group(0, false, false, false);
            set_leg_group(1, false, false, false);
            ESP_LOGI("Gait Control", "Gait BACKWARD C");
            break;
        case GAIT_BACKWARD_D:
            set_leg_group(0, false, false, false);
            set_leg_group(1, true, false, false);
            ESP_LOGI("Gait Control", "Gait BACKWARD D");
            break;

        // Sideways movement gaits
        case GAIT_LEFT_A:
            set_leg_group(0, false, false, true);
            set_leg_group(1, false, false, false);
            ESP_LOGI("Gait Control", "Gait LEFT A");
            break;
        case GAIT_LEFT_B:
            set_leg_group(0, true, false, true);
            set_leg_group(1, false, false, false);
            ESP_LOGI("Gait Control", "Gait LEFT B");
            break;
        case GAIT_RIGHT_A:
            set_leg_group(0, false, false, false);
            set_leg_group(1, false, false, true);
            ESP_LOGI("Gait Control", "Gait RIGHT A");
            break;
        case GAIT_RIGHT_B:
            set_leg_group(0, false, false, false);
            set_leg_group(1, true, false, true);
            ESP_LOGI("Gait Control", "Gait RIGHT B");
            break;

        default:
            ESP_LOGE("Gait Control", "Invalid gait pattern");
            break;
        }

        gait_pattern++;
        if (gait_pattern == 12) // Total number of gait patterns now
        {
            gait_pattern = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void servo_control_task(void *pvParameters)
{
    while (1)
    {
        ServoMessage msg;

        if (xQueueReceive(xServoQueue, &msg, portMAX_DELAY) == pdPASS)
        {
            HexapodLegServoDegree degrees = hexapod_leg_position_to_servo_degrees(msg.coord);
            set_leg_angle(msg.leg_id, degrees);
            ESP_LOGI("Servo Control", "Servo message received for leg %x: (X: %lf, Y: %lf, Z: %lf)", msg.leg_id, msg.coord.x, msg.coord.y, msg.coord.z);
            ESP_LOGI("Servo Control", "Servo angles set for leg %x: (A: %d, B: %d, C: %d)", msg.leg_id, degrees.a, degrees.b, degrees.c);
        }
    }
}

void app_main()
{
	hexapod_init();

    // xSensorQueue = xQueueCreate(10, sizeof(message_t));
    xServoQueue = xQueueCreate(32, sizeof(ServoMessage));
    if (/*xSensorQueue == NULL || */ xServoQueue == NULL)
    {
        ESP_LOGE("Driver", "Failed to create message queues\n");
        return;
    }

    ESP_LOGI("Driver", "Servo message queue created\n");


    // 创建传感器任务
    // xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 5, NULL);

    xTaskCreate(gait_control_task, "gait_control_task", 4096, NULL, 5, NULL);
    xTaskCreate(servo_control_task, "servo_control_task", 4096, NULL, 5, NULL);

    // idle
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
