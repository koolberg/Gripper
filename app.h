#ifndef APP_H
#define APP_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// ── Packet Definition ─────────────────────────────────────
// 23 bytes total:
// 0xAA(1) + 5 floats(20) + flags(1) + checksum(1)
#define PACKET_SIZE     23
#define START_BYTE      0xAA

// ── Packet Flags ──────────────────────────────────────────
#define FLAG_NONE       0x00
#define FLAG_GRIP_LOCK  0x01
#define FLAG_EMERGENCY  0x02
#define FLAG_HOME       0x04

// ── Servo PWM Pulse Limits ────────────────────────────────
// Timer: Prescaler=79, Period=19999 → 50Hz @ 80MHz
// 1000 ticks = 1ms = 0°
// 2000 ticks = 2ms = 180°
// 1500 ticks = 1.5ms = 90° (center)
#define SERVO_PULSE_MIN     1000
#define SERVO_PULSE_MAX     2000
#define SERVO_PULSE_CENTER  1500

// ── Servo Safety Limits (degrees) ────────────────────────
#define JOINT1_MIN   10
#define JOINT1_MAX   170
#define JOINT2_MIN   10
#define JOINT2_MAX   170
#define JOINT3_MIN   10
#define JOINT3_MAX   170
#define JOINT4_MIN   10
#define JOINT4_MAX   170
#define JOINT5_MIN   10
#define JOINT5_MAX   170
#define JOINT6_MIN   10     // gripper fully open
#define JOINT6_MAX   170    // gripper fully closed

// ── Stepper Settings ──────────────────────────────────────
#define STEPPER_STEP_DELAY_US   800   // microseconds between steps
                                       // lower = faster rotation
#define STEPPER_STEPS_PER_DEG   2     // half step = 400 steps/rev
                                       // 400/360 = 1.11 steps/deg

// ── Smoothing Factor ─────────────────────────────────────
#define SMOOTH_ALPHA    0.30f

// ── HAL Handles from main.c ───────────────────────────────
extern TIM_HandleTypeDef   htim2;
extern TIM_HandleTypeDef   htim3;
extern UART_HandleTypeDef  huart1;
extern UART_HandleTypeDef  huart2;

// ── Shared Variables ──────────────────────────────────────
extern uint8_t  rxBuffer[PACKET_SIZE];
extern uint8_t  rxByte;
extern uint8_t  rxIndex;

extern float    joint1Angle;
extern float    joint2Angle;
extern float    joint3Angle;
extern float    joint4Angle;
extern float    joint5Angle;
extern float    joint6Angle;

extern float    stepperYaw;
extern uint8_t  gripLocked;
extern uint32_t lastPrint;

// ── Function Declarations ─────────────────────────────────
void    App_Init(void);
void    App_Loop(void);

float   bytesToFloat(uint8_t *buf);
uint8_t calcChecksum(uint8_t *buf, uint8_t len);
float   smoothAngle(float current, float target);
float   clampAngle(float angle, float minVal, float maxVal);
void    setServoAngle(TIM_HandleTypeDef *htim,
                      uint32_t channel,
                      float angle);
void    allServosHome(void);
void    stepperMove(float targetYaw);
void    HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif // APP_H
