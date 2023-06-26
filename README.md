# EMM42 v4.2 library for ESP32 (ESP-IDF)

## Notes
* Declare how many motors you want to use by changing `EMM42_MOTOR_N` in `emm42_servo.c`.
* It is advisory to wait a bit after using `emm42_servo_move` and `emm42_servo_uart_turn`, because of UART trash.

## To do list:
* add support for other checksums
* add checking for correct checksum

## Sources
https://github.com/Macrobase-tech/Emm42