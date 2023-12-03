#ifndef PICO_MICRO_ROS_H
#define PICO_MICRO_ROS_H

#define WHEEL_RADIUS 0.042
#define GEAR_RATIO 78.0
#define ENCODER_RES 20.0
#define WHEEL_BASE 0.15

#define LEFT_ENC_POL 1
#define RIGHT_ENC_POL -1
#define LEFT_MOTOR_POL -1
#define RIGHT_MOTOR_POL 1

#define SLOPE_L 1.3883
#define SLOPE_R 1.424
#define INTERCEPT_L 0.1193
#define INTERCEPT_R 0.10046

#define LEFT_MOTOR_CHANNEL 1
#define RIGHT_MOTOR_CHANNEL 3

#define PI 3.142

typedef struct
{
    float x;
    float y;
    float yaw;
} odom_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;


#endif