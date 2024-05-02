#ifndef _MOTOR_STRUCT_H_
#define _MOTOR_STRUCT_H_
#include <stdint.h>
#pragma pack(1)
typedef struct motor_cmd_struct
{
    uint8_t ID;
    int32_t position;
    int32_t velocity;
    int32_t torque;
    int16_t Kp;
    int16_t Kd;
} motor_cmd_t;
typedef struct motor_back_struct
{
    uint8_t ID;
    float position;
    float velocity;
    float torque;
} motor_back_t;
typedef struct motor_back_raw_struct
{
    uint8_t ID;
    int32_t position;
    int32_t velocity;
    int32_t torque;
} motor_back_raw_t;
#pragma pack()
#endif