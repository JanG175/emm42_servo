# EMM42 v4.2 library for ESP32 (ESP-IDF)

## Notes
* Declare how many motors you want to use by changing `EMM42_MOTOR_N` in `emm42_servo.c`.
* It is advisory to wait a bit (10 ms) after using `emm42_servo_move` and `emm42_servo_uart_turn`, because of UART trash.
* One full rotation is 200 steps (in 1 microstep mode).

## To do list:
* add support for other checksums

## Sources
https://github.com/Macrobase-tech/Emm42