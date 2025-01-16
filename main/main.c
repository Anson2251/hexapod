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
    // GAIT_E,
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
    uint16_t group_id;
    Coordination coord;
} ServoMessage;

const int8_t SERVO_GPIOs[18] = {
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    26,
    27,
    28,
};

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

void set_leg_group(int8_t num, bool raise, bool forward)
{
    for (int8_t i = 0; i < 3; i++)
    {
        ServoMessage *msg = (ServoMessage *)pvPortMalloc(sizeof(ServoMessage));
        if (msg == NULL)
        {
            printf("Failed to allocate memory for message\n");
            continue;
        }

        msg->coord.x = Step ? forward : 0;
        msg->coord.y = Wingspan;
        msg->coord.z = -(raise ? CoreHeight + LegRiseHeight : CoreHeight);
        msg->group_id = num + i;

        if (xQueueSend(xServoQueue, &msg, portMAX_DELAY) != pdPASS)
        {
            printf("Failed to send servo message to queue\n");
            vPortFree(msg);
        }
    }
}

void gait_control_task(void *pvParameters)
{
    int8_t gait_pattern = 0;
    while (1)
    {
        switch (gait_pattern)
        {
        case GAIT_A:
            // Gait A:
            // - Leg group 0: Lowered and not moving forward (stance phase)
            // - Leg group 1: Lowered and moving forward (swing phase)
            set_leg_group(0, false, false);
            set_leg_group(1, false, true);
            break;

        case GAIT_B:
            // Gait B:
            // - Leg group 0: Raised and moving forward (swing phase)
            // - Leg group 1: Lowered and not moving forward (stance phase)
            set_leg_group(0, true, true);
            set_leg_group(1, false, false);
            break;

        case GAIT_C:
            // Gait C:
            // - Leg group 0: Lowered and moving forward (swing phase)
            // - Leg group 1: Lowered and not moving forward (stance phase)
            set_leg_group(0, false, true);
            set_leg_group(1, false, false);
            break;

        case GAIT_D:
            // Gait D:
            // - Leg group 0: Lowered and not moving forward (stance phase)
            // - Leg group 1: Raised and moving forward (swing phase)
            set_leg_group(0, false, false);
            set_leg_group(1, true, true);
            break;
        // case GAIT_E:
        //     set_leg_group(0, false, false);
        //     set_leg_group(1, false, true);
        //     break;
        default:
            break;
        }

        gait_pattern++;
        if (gait_pattern == 4)
        {
            gait_pattern = 0;
        }
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
            set_servo_angle(msg.group_id * 3 + 0, degrees.a);
            set_servo_angle(msg.group_id * 3 + 1, degrees.b);
            set_servo_angle(msg.group_id * 3 + 2, degrees.c);
        }
    }
}

void app_main()
{
    i2c_master_init();
    ESP_LOGI("PCA9685", "I2C initialized");

    pca9685_init();
    ESP_LOGI("PCA9685", "PCA9685 initialized");

    printf("All drivers initialized\n");

    // xSensorQueue = xQueueCreate(10, sizeof(message_t));
    xServoQueue = xQueueCreate(10, sizeof(ServoMessage));
    if (/*xSensorQueue == NULL || */ xServoQueue == NULL)
    {
        printf("Failed to create message queues\n");
        return;
    }

    // 创建传感器任务
    // xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 5, NULL);

    xTaskCreate(gait_control_task, "gait_control_task", 2048, NULL, 5, NULL);

    xTaskCreate(servo_control_task, "servo_control_task", 2048, NULL, 5, NULL);

    // 主任务不需要做其他事情
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}