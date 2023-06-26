/**
 * @file mks_servo.c
 * @author JanG175
 * @brief EMM42 SERVO SERIAL PROTOCOL LIBRARY
 * 
 * @copyright All rigths reserved (R) 2023
 */

#include <stdio.h>
#include "emm42_servo.h"

#define EMM42_MOTOR_N 2 // declare how many motors do you want to use

static bool abort_on = true; // if true, abort on error

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static gptimer_handle_t gptimer[EMM42_MOTOR_N];
static int64_t steps_left[EMM42_MOTOR_N];

static const uint32_t timer_N = EMM42_MOTOR_N; // number of timers

static const char* TAG = "emm42_servo";

static uint8_t crc = EMM42_CRC_0x6B; // CRC mode


/**
 * @brief calculate CRC for EMM42 UART communication
 * 
 * @param datagram pointer to datagram
 * @param len length of datagram
 * 
 * @return calculated CRC
 */
static uint8_t emm42_servo_uart_calc_CRC(uint8_t* datagram, uint32_t len, uint8_t crc_type)
{
    uint8_t crc = 0x00;

    if (crc_type == EMM42_CRC_0x6B)
        crc = 0x6B;

    return crc;
}


/**
 * @brief check if CRC is correct
 * 
 * @param datagram pointer to datagram
 * @param len length of datagram
 * 
 * @return true - if CRC is correct; false - otherwise
 */
static bool emm42_servo_uart_check_CRC(uint8_t* datagram, uint32_t len, uint8_t crc_type)
{
    uint8_t crc = emm42_servo_uart_calc_CRC(datagram, len, crc_type);

    if (crc == datagram[len - 1])
        return true;
    else
        return false;
}


/**
 * @brief write datagram to register via UART
 * 
 * @param emm42_conf struct with EMM42 connection parameters
 * @param datagram pointer to datagram
 * @param len length of datagram
 */
static void emm42_servo_uart_send(emm42_conf_t emm42_conf, uint8_t* datagram, uint8_t len)
{
    uart_write_bytes(emm42_conf.uart, (const uint8_t*)datagram, len);
    ESP_ERROR_CHECK(uart_wait_tx_done(emm42_conf.uart, EMM42_UART_TIMEOUT_MS));
}


/**
 * @brief read datagram from register via UART
 * 
 * @param emm42_conf struct with EMM42 connection parameters
 * @param datagram pointer to datagram
 * @param len length of datagram
 */
static void emm42_servo_uart_recv(emm42_conf_t emm42_conf, uint8_t* datagram, uint8_t len)
{
    uint32_t buf = 0;
    uint8_t data[len];

    buf = uart_read_bytes(emm42_conf.uart, data, len, EMM42_UART_TIMEOUT_MS);
    uart_flush(emm42_conf.uart);

    if (buf == len)
    {
        for (uint32_t i = 0; i < buf; i++)
        {
            datagram[i] = data[i];
        }
    }
    else
    {
        ESP_LOGE(TAG, "UART read error");

        for (uint32_t i = 0; i < len; i++)
            datagram[i] = 0;
    }
}


/**
 * @brief callback function for timers
 * 
 * @param timer timer handle
 * @param edata event data
 * @param user_ctx user context
 * @return true - if high priority task was woken up; false - otherwise
 */
static bool clk_timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* user_ctx)
{
    BaseType_t high_task_awoken = pdFALSE;

    emm42_cb_arg_t cb_arg = *(emm42_cb_arg_t*)user_ctx;
    gpio_num_t step_pin = cb_arg.step_pin;
    uint32_t motor_num = cb_arg.motor_num;

    if (steps_left[motor_num] > 0)
    {
        if (gpio_get_level(step_pin) == 0)
            gpio_set_level(step_pin, 1);
        else
            gpio_set_level(step_pin, 0);

        portENTER_CRITICAL(&spinlock);
        steps_left[motor_num]--;
        portEXIT_CRITICAL(&spinlock);
    }
    else
        ESP_ERROR_CHECK(gptimer_stop(timer));

    return (high_task_awoken == pdTRUE);
}


/**
 * @brief initialize EMM42 UART and timers
 * 
 * @param emm42_conf struct with EMM42 connection parameters
 */
void emm42_servo_init(emm42_conf_t emm42_conf)
{
    // configure UART
    uart_config_t uart_config = {
        .baud_rate = emm42_conf.baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT
    };

    if (uart_is_driver_installed(emm42_conf.uart) == true)
        ESP_ERROR_CHECK(uart_driver_delete(emm42_conf.uart));

    ESP_ERROR_CHECK(uart_driver_install(emm42_conf.uart, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(emm42_conf.uart, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(emm42_conf.uart, emm42_conf.tx_pin, emm42_conf.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    emm42_cb_arg_t* cb_arg = malloc(sizeof(emm42_cb_arg_t) * timer_N); // allocate memory for callback arguments

    for (uint32_t i = 0; i < timer_N; i++)
    {
        // configure step and dir pins

        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
        io_conf.pin_bit_mask = ((1 << emm42_conf.step_pin[i]) | (1 << emm42_conf.dir_pin[i]) | (1 << emm42_conf.en_pin[i]));
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);

        emm42_servo_enable(emm42_conf, i, 0);
        emm42_servo_set_dir(emm42_conf, i, EMM42_CW_DIR);

        // configure timers

        gptimer_config_t timer_config = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000 // 1 us
        };

        ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer[i]));

        gptimer_alarm_config_t alarm_config = {
            .alarm_count = 1000000, // 1 s
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true
        };

        ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer[i], &alarm_config));

        gptimer_event_callbacks_t timer_cbs = {
            .on_alarm = clk_timer_callback
        };

        cb_arg[i].step_pin = emm42_conf.step_pin[i];
        cb_arg[i].motor_num = i;

        ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer[i], &timer_cbs, (void*)&cb_arg[i]));

        ESP_ERROR_CHECK(gptimer_enable(gptimer[i]));

        emm42_servo_enable(emm42_conf, i, 1); // enable motor
    }
}


// deinit EMM42 UART and timers
void emm42_servo_deinit(emm42_conf_t emm42_conf)
{
    for (uint32_t i = 0; i < timer_N; i++)
    {
        ESP_ERROR_CHECK(gptimer_disable(gptimer[i]));
        ESP_ERROR_CHECK(gptimer_del_timer(gptimer[i]));
    }

    ESP_ERROR_CHECK(uart_driver_delete(emm42_conf.uart));
}


/**
 * @brief set enable pin
 * 
 * @param emm42_conf struct with EMM42 connection parameters
 * @param motor_num motor number
 * @param enable 0 - disable, 1 - enable
 */
void emm42_servo_enable(emm42_conf_t emm42_conf, uint32_t motor_num, uint32_t enable)
{
    if (enable == 0)
        gpio_set_level(emm42_conf.en_pin[motor_num], 0);
    else if (enable == 1)
        gpio_set_level(emm42_conf.en_pin[motor_num], 1);
}


/**
 * @brief set direction pin
 * 
 * @param emm42_conf struct with EMM42 connection parameters
 * @param motor_num motor number
 * @param dir direction (EMM42_CW_DIR or EMM42_CCW_DIR)
 */
void emm42_servo_set_dir(emm42_conf_t emm42_conf, uint32_t motor_num, uint32_t dir)
{
    if (dir == EMM42_CW_DIR)
        gpio_set_level(emm42_conf.dir_pin[motor_num], 0);
    else if (dir == EMM42_CCW_DIR)
        gpio_set_level(emm42_conf.dir_pin[motor_num], 1);
}


/**
 * @brief set period for step signal
 * 
 * @param motor_num motor number
 * @param period_us period in us
 */
void emm42_servo_set_period(uint32_t motor_num, uint32_t period_us)
{
    if (period_us < 500)
        ESP_LOGW(TAG, "Period is too small, motor might not work properly!");

    period_us = period_us / 2; // 1 period = 2 gpio switches

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = period_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer[motor_num], &alarm_config));
}


/**
 * @brief start/stop step signal
 * 
 * @param emm42_conf struct with EMM42 connection parameters
 * @param motor_num motor number
 * @param start 0 - stop, 1 - start
 */
void emm42_servo_start(emm42_conf_t emm42_conf, uint32_t motor_num, uint32_t start)
{
    if (start == 0)
    {
        ESP_ERROR_CHECK(gptimer_stop(gptimer[motor_num]));
        gpio_set_level(emm42_conf.step_pin[motor_num], 0);
    }
    else if (start == 1)
    {
        ESP_ERROR_CHECK(gptimer_start(gptimer[motor_num]));
    }
}


/**
 * @brief move all motors by desired number of steps with desired period and direction (sign in steps variable)
 * 
 * @param emm42_conf struct with EMM42 connection parameters
 * @param steps array of steps for each motor
 * @param period_us array of periods for each motor
 */
void emm42_servo_step_move(emm42_conf_t emm42_conf, int64_t* steps, uint32_t* period_us)
{
    for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
    {
        // set direction
        if (steps[motor_num] < 0)
        {
            emm42_servo_set_dir(emm42_conf, motor_num, 0);
            steps[motor_num] = -steps[motor_num];
        }
        else
            emm42_servo_set_dir(emm42_conf, motor_num, 1);

        // set period
        emm42_servo_set_period(motor_num, period_us[motor_num]);

        // set number of steps
        portENTER_CRITICAL(&spinlock);
        steps_left[motor_num] = 2 * steps[motor_num];
        portEXIT_CRITICAL(&spinlock);
    }

    // start all motors one by one
    for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
        emm42_servo_start(emm42_conf, motor_num, 1);

    bool end_wait = false;

    // wait until all motors stop
    while (end_wait == false)
    {
        int64_t steps_left_status = 0;

        for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
            steps_left_status = steps_left_status + steps_left[motor_num];

        if (steps_left_status <= 0)
            end_wait = true;

        vTaskDelay(1);
    }

    // // for debug
    // for (uint32_t motor_num = 0; motor_num < timer_N; motor_num++)
    //     ESP_LOGI(TAG, "%lu: %lld", motor_num, steps_left[motor_num] / 2);
}


/**
 * @brief calibrate encoder
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 */
void emm42_servo_uart_calibrate_encoder(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    datagram[0] = address;
    datagram[1] = 0x06;
    datagram[2] = 0x45;
    datagram[3] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);

    // wait for 30 sec for encoder calibration
    vTaskDelay(30000 / portTICK_PERIOD_MS);
}


/**
 * @brief set 0 position
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 */
void emm42_servo_uart_set_zero_pos(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    datagram[0] = address;
    datagram[1] = 0x0A;
    datagram[2] = 0x6D;
    datagram[3] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);
}


/**
 * @brief read encoder value
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * 
 * @return encoder value
 */
float emm42_servo_uart_read_encoder(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 3;
    uint8_t datagram[len_w];
    uint8_t len_r = 4;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_ENCODER_READ;
    datagram[2] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        emm42_servo_uart_recv(emm42_conf, response, len_r);
        cnt++;
    } while ((response[0] != address || emm42_servo_uart_check_CRC(datagram, len_r, crc) != false) && (cnt < EMM42_UART_MAX_REPEAT));

    if (cnt >= EMM42_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");

        if (abort_on == true)
            abort();
    }

    uint16_t value = response[1] << 8 | response[2];

    float encoder_value = (float)value * 360.0f / 65536.0f;

    return encoder_value;
}


/**
 * @brief read the number of pulses received.
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * 
 * @return number of pulses
 */
int32_t emm42_servo_uart_read_pulses(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 3;
    uint8_t datagram[len_w];
    uint8_t len_r = 6;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_PULSE_READ;
    datagram[2] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        emm42_servo_uart_recv(emm42_conf, response, len_r);
        cnt++;
    } while ((response[0] != address || emm42_servo_uart_check_CRC(datagram, len_r, crc) != false) && (cnt < EMM42_UART_MAX_REPEAT));

    if (cnt >= EMM42_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");

        if (abort_on == true)
            abort();
    }

    int32_t pulses = response[1] << 24 | response[2] << 16 | response[3] << 8 | response[4];

    return pulses;
}


/**
 * @brief read current motor position
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * 
 * @return current motor position
 */
float emm42_servo_uart_read_motor_pos(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 3;
    uint8_t datagram[len_w];
    uint8_t len_r = 6;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_MOTOR_POS_READ;
    datagram[2] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        emm42_servo_uart_recv(emm42_conf, response, len_r);
        cnt++;
    } while ((response[0] != address || emm42_servo_uart_check_CRC(datagram, len_r, crc) != false) && (cnt < EMM42_UART_MAX_REPEAT));

    if (cnt >= EMM42_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");

        if (abort_on == true)
            abort();
    }

    int32_t value = response[1] << 24 | response[2] << 16 | response[3] << 8 | response[4];

    float motor_pos = (float)value * 360.0f / 65536.0f;

    return motor_pos;
}


/**
 * @brief read the error of the motor shaft angle
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * 
 * @return error of the motor shaft angle
 */
float emm42_servo_uart_read_motor_shaft_error(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 3;
    uint8_t datagram[len_w];
    uint8_t len_r = 4;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_SHAFT_ERROR_READ;
    datagram[2] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        emm42_servo_uart_recv(emm42_conf, response, len_r);
        cnt++;
    } while ((response[0] != address || emm42_servo_uart_check_CRC(datagram, len_r, crc) != false) && (cnt < EMM42_UART_MAX_REPEAT));

    if (cnt >= EMM42_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");

        if (abort_on == true)
            abort();
    }

    int16_t value = response[1] << 8 | response[2];

    float shaft_error = (float)value * 360.0f / 65536.0f;

    return shaft_error;
}


/**
 * @brief read enable status
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * @return enable status
 */
uint8_t emm42_servo_uart_read_enable(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 3;
    uint8_t datagram[len_w];
    uint8_t len_r = 3;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_ENABLE_READ;
    datagram[2] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        emm42_servo_uart_recv(emm42_conf, response, len_r);
        cnt++;
    } while ((response[0] != address || emm42_servo_uart_check_CRC(datagram, len_r, crc) != false) && (cnt < EMM42_UART_MAX_REPEAT));

    if (cnt >= EMM42_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");

        if (abort_on == true)
            abort();
    }

    return response[1];
}


/**
 * @brief release stall protection
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 */
void emm42_servo_uart_release_stall_protection(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    datagram[0] = address;
    datagram[1] = 0x0E;
    datagram[2] = 0x52;
    datagram[3] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);
}


/**
 * @brief read stall flag
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * 
 * @return stall flag status (0 - no stall, 1 - stall)
 */
uint8_t emm42_servo_uart_read_stall_flag(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 3;
    uint8_t datagram[len_w];
    uint8_t len_r = 3;
    uint8_t response[len_r];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    for (uint32_t i = 0; i < len_r; i++)
        response[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_SHAFT_READ;
    datagram[2] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);

    uint32_t cnt = 0;

    do
    {
        emm42_servo_uart_recv(emm42_conf, response, len_r);
        cnt++;
    } while ((response[0] != address || emm42_servo_uart_check_CRC(datagram, len_r, crc) != false) && (cnt < EMM42_UART_MAX_REPEAT));

    if (cnt >= EMM42_UART_MAX_REPEAT)
    {
        ESP_LOGE(TAG, "UART read timeout");

        if (abort_on == true)
            abort();
    }

    return response[1];
}


/**
 * @brief set microstep divider
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * @param mstep microstep divider
 */
void emm42_servo_uart_set_mstep(emm42_conf_t emm42_conf, uint8_t address, uint16_t mstep)
{
    if (mstep < 1)
    {
        mstep = 1;
        ESP_LOGW(TAG, "Invalid mstep value. Set to 1.");
    }
    else if (mstep > 256)
    {
        mstep = 0;
        ESP_LOGW(TAG, "Invalid mstep value. Set to 256.");
    }
    else if (mstep == 256)
        mstep = 0;

    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_MSTEP_FUNC;
    datagram[2] = (uint8_t)mstep;
    datagram[3] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);
}


/**
 * @brief set new address
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * @param new_address new emm42 slave address
 */
void emm42_servo_uart_set_address(emm42_conf_t emm42_conf, uint8_t address, uint8_t new_address)
{
    if (new_address < 1)
    {
        new_address = 1;
        ESP_LOGW(TAG, "Invalid address. Set to 1.");
    }
    else if (new_address > 247)
    {
        new_address = 247;
        ESP_LOGW(TAG, "Invalid address. Set to 247.");
    }

    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_ADDR_FUNC;
    datagram[2] = new_address;
    datagram[3] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);
}


/**
 * @brief set enable status
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * @param en_status enable status (0 - enable on low, 1 - enable on high)
 */
void emm42_servo_uart_set_enable(emm42_conf_t emm42_conf, uint8_t address, uint8_t en_status)
{
    if (en_status != 1 || en_status != 0)
    {
        en_status = 1;
        ESP_LOGW(TAG, "Invalid enable status. Set to 1.");
    }

    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_EN_FUNC;
    datagram[2] = en_status;
    datagram[3] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);
}


/**
 * @brief turn endlessly with speed and acceleration
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * @param speed desired speed (MSB - direction)
 * @param accel desired acceleration (255 - run without acceleration)
 */
void emm42_servo_uart_turn(emm42_conf_t emm42_conf, uint8_t address, int16_t speed, uint8_t accel)
{
    if (abs(speed) > 0x4FF)
    {
        speed = speed / abs(speed) * 0x4FF;
        ESP_LOGW(TAG, "Too high speed value.");
    }

    if (speed < 0)
    {
        speed = -speed;
        speed = speed | (1 << 15);
    }

    uint8_t len_w = 6;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    uint8_t speedH = (uint8_t)(speed >> 8);
    uint8_t speedL = (uint8_t)speed;

    datagram[0] = address;
    datagram[1] = EMM42_TURN_FUNC;
    datagram[2] = speedH;
    datagram[3] = speedL;
    datagram[4] = accel;
    datagram[5] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);
}


/**
 * @brief store endless turn params for startup
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 */
void emm42_servo_uart_store_turn_params(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_STORE_CLEAR_FUNC;
    datagram[2] = 0xC8;
    datagram[3] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);
}


/**
 * @brief clear endless turn params for startup
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 */
void emm42_servo_uart_clear_turn_params(emm42_conf_t emm42_conf, uint8_t address)
{
    uint8_t len_w = 4;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    datagram[0] = address;
    datagram[1] = EMM42_STORE_CLEAR_FUNC;
    datagram[2] = 0xCA;
    datagram[3] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);
}


/**
 * @brief move by desired pulses number with speed and acceleration
 * 
 * @param emm42_conf struct with emm42 configs
 * @param address emm42 slave address
 * @param speed desired speed (MSB - direction)
 * @param accel desired acceleration (255 - run without acceleration)
 * @param pulse desired position (3 bytes)
 */
void emm42_servo_uart_move(emm42_conf_t emm42_conf, uint8_t address, int16_t speed, uint8_t accel, uint32_t pulse)
{
    if (abs(speed) > 0x4FF)
    {
        speed = speed / abs(speed) * 0x4FF;
        ESP_LOGW(TAG, "Too high speed value.");
    }

    if (pulse > 0xFFFFFF)
    {
        pulse = 0xFFFFFF;
        ESP_LOGW(TAG, "Too high pulse value.");
    }

    if (speed < 0)
    {
        speed = -speed;
        speed = speed | (1 << 15);
    }

    uint8_t len_w = 9;
    uint8_t datagram[len_w];

    for (uint32_t i = 0; i < len_w; i++)
        datagram[i] = 0;

    uint8_t speedH = (uint8_t)(speed >> 8);
    uint8_t speedL = (uint8_t)speed;

    uint8_t pulseH = (uint8_t)(pulse >> 16);
    uint8_t pulseM = (uint8_t)(pulse >> 8);
    uint8_t pulseL = (uint8_t)pulse;

    datagram[0] = address;
    datagram[1] = EMM42_MOVE_FUNC;
    datagram[2] = speedH;
    datagram[3] = speedL;
    datagram[4] = accel;
    datagram[5] = pulseH;
    datagram[6] = pulseM;
    datagram[7] = pulseL;
    datagram[8] = emm42_servo_uart_calc_CRC(datagram, len_w, crc);

    emm42_servo_uart_send(emm42_conf, datagram, len_w);
}