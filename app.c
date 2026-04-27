#include "app.h"

// ── Variables ─────────────────────────────────────────────
uint8_t  rxBuffer[PACKET_SIZE];
uint8_t  rxByte;
uint8_t  rxIndex    = 0;

float    joint1Angle = 90.0f;   // base side to side
float    joint2Angle = 90.0f;   // base forward/back
float    joint3Angle = 90.0f;   // gripper up/down
float    joint4Angle = 90.0f;   // wrist rotation
float    joint5Angle = 90.0f;   // gripper connector
float    joint6Angle = 90.0f;   // gripper open/close

float    stepperYaw  = 0.0f;    // current stepper yaw
uint8_t  gripLocked  = 0;       // grip lock state
uint32_t lastPrint   = 0;

// ─────────────────────────────────────────────────────────
//  Helper Functions
// ─────────────────────────────────────────────────────────

// Convert 4 big-endian bytes to float
float bytesToFloat(uint8_t *buf) {
    float val;
    uint8_t tmp[4] = {buf[3], buf[2], buf[1], buf[0]};
    memcpy(&val, tmp, 4);
    return val;
}

// XOR checksum
uint8_t calcChecksum(uint8_t *buf, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) cs ^= buf[i];
    return cs;
}

// Smooth low pass filter
float smoothAngle(float current, float target) {
    return current + SMOOTH_ALPHA * (target - current);
}

// Clamp angle to safe limits
float clampAngle(float angle, float minVal, float maxVal) {
    if (angle < minVal) return minVal;
    if (angle > maxVal) return maxVal;
    return angle;
}

// Convert degrees to PWM pulse and write to servo
void setServoAngle(TIM_HandleTypeDef *htim,
                   uint32_t channel,
                   float angle) {
    uint32_t pulse = (uint32_t)(SERVO_PULSE_MIN +
                     (angle / 180.0f) *
                     (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

// Move all joints to home position (90 degrees center)
void allServosHome(void) {
    joint1Angle = 90.0f;
    joint2Angle = 90.0f;
    joint3Angle = 90.0f;
    joint4Angle = 90.0f;
    joint5Angle = 90.0f;
    joint6Angle = 90.0f;

    setServoAngle(&htim2, TIM_CHANNEL_1, joint1Angle);
    setServoAngle(&htim2, TIM_CHANNEL_2, joint2Angle);
    setServoAngle(&htim2, TIM_CHANNEL_3, joint3Angle);
    setServoAngle(&htim2, TIM_CHANNEL_4, joint4Angle);
    setServoAngle(&htim3, TIM_CHANNEL_1, joint5Angle);
    setServoAngle(&htim3, TIM_CHANNEL_2, joint6Angle);
}

// Move stepper to match target yaw
void stepperMove(float targetYaw) {
    float diff = targetYaw - stepperYaw;

    // Ignore tiny movements
    if (diff > -1.0f && diff < 1.0f) return;

    // Set direction
    if (diff > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        diff = -diff;
    }

    // Calculate steps needed
    uint32_t steps = (uint32_t)(diff * STEPPER_STEPS_PER_DEG);

    // Pulse STEP pin
    for (uint32_t i = 0; i < steps; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        DWT->CYCCNT = 0;
        while (DWT->CYCCNT < (80 * STEPPER_STEP_DELAY_US / 2));
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        while (DWT->CYCCNT < (80 * STEPPER_STEP_DELAY_US));
    }

    stepperYaw = targetYaw;
}

// ─────────────────────────────────────────────────────────
//  UART Interrupt Callback
// ─────────────────────────────────────────────────────────

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        // Sync on start byte 0xAA
        if (rxIndex == 0 && rxByte != START_BYTE) {
            HAL_UART_Receive_IT(&huart1, &rxByte, 1);
            return;
        }

        rxBuffer[rxIndex++] = rxByte;

        if (rxIndex >= PACKET_SIZE) {
            rxIndex = 0;

            // Validate checksum
            // Checksum covers bytes 1 to 21 (skip start + checksum)
            uint8_t expected = calcChecksum(&rxBuffer[1],
                                            PACKET_SIZE - 2);

            if (rxBuffer[PACKET_SIZE - 1] == expected) {

                // Extract 5 floats from packet
                float hand_yaw      = bytesToFloat(&rxBuffer[1]);
                float hand_pitch    = bytesToFloat(&rxBuffer[5]);
                float hand_roll     = bytesToFloat(&rxBuffer[9]);
                float forearm_pitch = bytesToFloat(&rxBuffer[13]);
                float finger_curl   = bytesToFloat(&rxBuffer[17]);
                uint8_t flags       = rxBuffer[21];

                // ── Handle Buttons ────────────────────────

                // Home position — return all joints to center
                if (flags & FLAG_HOME) {
                    allServosHome();
                    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
                    return;
                }

                // Emergency release — fully open gripper
                if (flags & FLAG_EMERGENCY) {
                    joint6Angle = JOINT6_MIN;
                    setServoAngle(&htim3, TIM_CHANNEL_2,
                                  joint6Angle);
                    gripLocked = 0;
                    HAL_UART_Receive_IT(&huart1, &rxByte, 1);
                    return;
                }

                // ── Map Sensor Data to Joints ─────────────

                // Joint 1 — base side to side
                // forearm roll controls side movement
                float t1 = clampAngle(90.0f + forearm_pitch,
                                      JOINT1_MIN, JOINT1_MAX);
                joint1Angle = smoothAngle(joint1Angle, t1);

                // Joint 2 — base forward/backward
                // forearm pitch controls forward/back
                float t2 = clampAngle(90.0f + hand_roll,
                                      JOINT2_MIN, JOINT2_MAX);
                joint2Angle = smoothAngle(joint2Angle, t2);

                // Joint 3 — gripper up/down
                // hand pitch controls up/down
                float t3 = clampAngle(90.0f + hand_pitch,
                                      JOINT3_MIN, JOINT3_MAX);
                joint3Angle = smoothAngle(joint3Angle, t3);

                // Joint 4 — wrist rotation
                // hand roll controls wrist rotation
                float t4 = clampAngle(90.0f + hand_yaw,
                                      JOINT4_MIN, JOINT4_MAX);
                joint4Angle = smoothAngle(joint4Angle, t4);

                // Joint 5 — gripper connector
                // combination of hand movements
                float t5 = clampAngle(90.0f + (hand_pitch * 0.5f),
                                      JOINT5_MIN, JOINT5_MAX);
                joint5Angle = smoothAngle(joint5Angle, t5);

                // Joint 6 — gripper open/close
                // only update if not locked
                if (!(flags & FLAG_GRIP_LOCK)) {
                    float t6 = clampAngle(
                               (finger_curl / 90.0f) * 170.0f,
                               JOINT6_MIN, JOINT6_MAX);
                    joint6Angle = smoothAngle(joint6Angle, t6);
                    gripLocked  = 0;
                } else {
                    // Lock is on — hold current position
                    gripLocked = 1;
                }

                // ── Write All Servos ──────────────────────
                setServoAngle(&htim2, TIM_CHANNEL_1, joint1Angle);
                setServoAngle(&htim2, TIM_CHANNEL_2, joint2Angle);
                setServoAngle(&htim2, TIM_CHANNEL_3, joint3Angle);
                setServoAngle(&htim2, TIM_CHANNEL_4, joint4Angle);
                setServoAngle(&htim3, TIM_CHANNEL_1, joint5Angle);
                setServoAngle(&htim3, TIM_CHANNEL_2, joint6Angle);

                // ── Move Stepper ──────────────────────────
                // hand yaw controls base rotation
                stepperMove(hand_yaw);

            } else {
                // Bad checksum
                char fail[] = "PKT FAIL\r\n";
                HAL_UART_Transmit(&huart2,
                                 (uint8_t*)fail,
                                 strlen(fail), 10);
            }
        }

        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}

// ─────────────────────────────────────────────────────────
//  App Init
// ─────────────────────────────────────────────────────────

void App_Init(void) {
    // Enable DWT cycle counter for stepper timing
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    // Start PWM on all servo channels
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // Center all servos on startup
    allServosHome();
    HAL_Delay(500);

    // Enable A4988 stepper driver (EN active LOW)


    // Start UART1 receive interrupt
    HAL_UART_Receive_IT(&huart1, &rxByte, 1);

    // Ready message to PuTTY
    char msg[] = "STM32 Full Arm Ready\r\n";
    HAL_UART_Transmit(&huart2,
                      (uint8_t*)msg,
                      strlen(msg), 100);
}

// ─────────────────────────────────────────────────────────
//  App Loop
// ─────────────────────────────────────────────────────────

void App_Loop(void) {
    uint32_t now = HAL_GetTick();
    if (now - lastPrint >= 1000) {
        lastPrint = now;

        char buf[128];
        snprintf(buf, sizeof(buf),
            "J1:%d J2:%d J3:%d J4:%d J5:%d J6:%d "
            "Yaw:%d Lock:%d\r\n",
            (int)joint1Angle,
            (int)joint2Angle,
            (int)joint3Angle,
            (int)joint4Angle,
            (int)joint5Angle,
            (int)joint6Angle,
            (int)stepperYaw,
            gripLocked);

        HAL_UART_Transmit(&huart2,
                         (uint8_t*)buf,
                         strlen(buf), 100);
    }
}
