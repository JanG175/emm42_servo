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

// EMM42 configs
#define EMM42_STEP_MODE_ENABLE        1 // uncomment to enable step mode
// #define EMM42_PC_RETURN            1 // uncomment if you want to return message to PC

// EMM42 control words
#define EMM42_COMMAND_VALID           0x02
#define EMM42_COMMAND_INVALID         0xEE

// EMM42 CRC
#define EMM42_CRC_0x6B                1
#define EMM42_CRC_XOR                 2
#define EMM42_CRC_8                   3

// EMM42 read parameters commands
#define EMM42_ENCODER_READ            0x30
#define EMM42_PULSE_READ              0x33
#define EMM42_MOTOR_POS_READ          0x36
#define EMM42_SHAFT_ERROR_READ        0x39
#define EMM42_ENABLE_READ             0x3A
#define EMM42_SHAFT_READ              0x3E
#define EMM42_GO_TO_ZERO_POS_READ     0x3F

// EMM42 parameters commands
#define EMM42_MSTEP_FUNC              0x84
#define EMM42_ADDR_FUNC               0xAE
#define EMM42_EN_FUNC                 0xF3
#define EMM42_TURN_FUNC               0xF6
#define EMM42_STORE_CLEAR_FUNC        0xFF
#define EMM42_MOVE_FUNC               0xFD

// EMM42 constants
#define EMM42_UART_TIMEOUT_MS         (100 / portTICK_PERIOD_MS)
#define EMM42_UART_MAX_REPEAT         10

#define EMM42_CW_DIR                  0
#define EMM42_CCW_DIR                 1
#define EMM42_FULL_ROT                200

typedef struct emm42_conf_t
{
    uart_port_t uart;
    uint32_t baudrate;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
#ifdef EMM42_STEP_MODE_ENABLE
    uint8_t motor_num;
    gpio_num_t* step_pin;
    gpio_num_t* dir_pin;
    gpio_num_t* en_pin;
#endif // EMM42_STEP_MODE_ENABLE
} emm42_conf_t;

typedef struct emm42_cb_arg_t
{
    gpio_num_t step_pin;
    uint8_t motor_num;

    uint64_t steps_left;
    uint64_t steps_total;
    uint64_t period_goal;

    double time_passed;
    double dt;
    double accel_s;
} emm42_cb_arg_t;


void emm42_servo_init(emm42_conf_t emm42_conf);

void emm42_servo_deinit(emm42_conf_t emm42_conf);

#ifdef EMM42_STEP_MODE_ENABLE
void emm42_servo_enable(emm42_conf_t emm42_conf, uint8_t motor_num, bool enable);

void emm42_servo_set_dir(emm42_conf_t emm42_conf, uint8_t motor_num, uint8_t cw);

void emm42_servo_set_period(uint8_t motor_num, uint64_t period_us);

void emm42_servo_start(emm42_conf_t emm42_conf, uint8_t motor_num, bool start);

void emm42_servo_step_move(emm42_conf_t emm42_conf, uint8_t motor_num, uint64_t steps, int64_t period_us, float accel_phase);
#endif // EMM42_STEP_MODE_ENABLE

void emm42_servo_uart_calibrate_encoder(emm42_conf_t emm42_conf, uint8_t address);

void emm42_servo_uart_set_zero_pos(emm42_conf_t emm42_conf, uint8_t address);

float emm42_servo_uart_read_encoder(emm42_conf_t emm42_conf, uint8_t address);

int32_t emm42_servo_uart_read_pulses(emm42_conf_t emm42_conf, uint8_t address);

float emm42_servo_uart_read_motor_pos(emm42_conf_t emm42_conf, uint8_t address);

float emm42_servo_uart_read_motor_shaft_error(emm42_conf_t emm42_conf, uint8_t address);

uint8_t emm42_servo_uart_read_enable(emm42_conf_t emm42_conf, uint8_t address);

void emm42_servo_uart_release_stall_protection(emm42_conf_t emm42_conf, uint8_t address);

uint8_t emm42_servo_uart_read_stall_flag(emm42_conf_t emm42_conf, uint8_t address);

void emm42_servo_uart_set_mstep(emm42_conf_t emm42_conf, uint8_t address, uint16_t mstep);

void emm42_servo_uart_set_address(emm42_conf_t emm42_conf, uint8_t address, uint8_t new_address);

void emm42_servo_uart_set_enable(emm42_conf_t emm42_conf, uint8_t address, uint8_t en_status);

void emm42_servo_uart_turn(emm42_conf_t emm42_conf, uint8_t address, int16_t speed, uint8_t accel);

void emm42_servo_uart_store_turn_params(emm42_conf_t emm42_conf, uint8_t address);

void emm42_servo_uart_clear_turn_params(emm42_conf_t emm42_conf, uint8_t address);

void emm42_servo_uart_move(emm42_conf_t emm42_conf, uint8_t address, int16_t speed, uint8_t accel, uint32_t pulse);