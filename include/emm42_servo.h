/**
 * @file mks_servo.h
 * @author JanG175
 * @brief EMM42 SERVO SERIAL PROTOCOL LIBRARY
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// EMM42 control words
#define COMMAND_VALID           0x02
#define COMMAND_INVALID         0xEE

// EMM42 CRC
#define CRC_0x6B                1
#define CRC_XOR                 2
#define CRC_8                   3

// EMM42 read parameters commands
#define ENCODER_READ            0x30
#define PULSE_READ              0x33
#define MOTOR_POS_READ          0x36
#define SHAFT_ERROR_READ        0x39
#define ENABLE_READ             0x3A
#define SHAFT_READ              0x3E
#define GO_TO_ZERO_POS_READ     0x3F

// EMM42 parameters commands
#define MSTEP_FUNC              0x84
#define ADDR_FUNC               0xAE
#define EN_FUNC                 0xF3
#define TURN_FUNC               0xF6
#define STORE_CLEAR_FUNC        0xFF
#define MOVE_FUNC               0xFD

// EMM42 configs
#define TIMER_GROUP TIMER_GROUP_0
#define TIMER_ID TIMER_0

#define UART_TIMEOUT_MS (100 / portTICK_PERIOD_MS)
#define UART_MAX_REPEAT 10

#define CW_DIR 0
#define CCW_DIR 1
#define FULL_ROT 200

typedef struct emm42_config_t
{
    uart_port_t uart;
    uint32_t baudrate;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t* step_pin;
    gpio_num_t* dir_pin;
    gpio_num_t* en_pin;
} emm42_config_t;

typedef struct cb_arg_t
{
    gpio_num_t step_pin;
    uint32_t motor_num;
} cb_arg_t;


void emm42_init(emm42_config_t emm42_config);

void emm42_deinit(emm42_config_t emm42_config);

void emm42_enable(emm42_config_t emm42_config, uint32_t motor_num, uint32_t enable);

void emm42_set_dir(emm42_config_t emm42_config, uint32_t motor_num, uint32_t cw);

void emm42_set_period(uint32_t motor_num, uint32_t period_us);

void emm42_start(emm42_config_t emm42_config, uint32_t motor_num, uint32_t start);

void emm42_step_move(emm42_config_t emm42_config, int64_t* steps, uint32_t* period_us);

void emm42_uart_calibrate_encoder(emm42_config_t emm42_config, uint8_t address);

void emm42_uart_set_zero_pos(emm42_config_t emm42_config, uint8_t address);

float emm42_uart_read_encoder(emm42_config_t emm42_config, uint8_t address);

int32_t emm42_uart_read_pulses(emm42_config_t emm42_config, uint8_t address);

float emm42_uart_read_motor_pos(emm42_config_t emm42_config, uint8_t address);

float emm42_uart_read_motor_shaft_error(emm42_config_t emm42_config, uint8_t address);

uint8_t emm42_uart_read_enable(emm42_config_t emm42_config, uint8_t address);

void emm42_uart_release_stall_protection(emm42_config_t emm42_config, uint8_t address);

uint8_t emm42_uart_read_stall_flag(emm42_config_t emm42_config, uint8_t address);

void emm42_uart_set_mstep(emm42_config_t emm42_config, uint8_t address, uint16_t mstep);

void emm42_uart_set_address(emm42_config_t emm42_config, uint8_t address, uint8_t new_address);

void emm42_uart_set_enable(emm42_config_t emm42_config, uint8_t address, uint8_t en_status);

void emm42_uart_turn(emm42_config_t emm42_config, uint8_t address, int16_t speed, uint8_t accel);

void emm42_uart_store_turn_params(emm42_config_t emm42_config, uint8_t address);

void emm42_uart_clear_turn_params(emm42_config_t emm42_config, uint8_t address);

void emm42_uart_move(emm42_config_t emm42_config, uint8_t address, int16_t speed, uint8_t accel, uint32_t pulse);