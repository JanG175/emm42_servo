# EMM42 v4.2 library for ESP32 (ESP-IDF)

## Notes
* Uncomment `#define EMM42_STEP_MODE_ENABLE 1` in `emm42_servo.h` in config section to enable step mode.
* Uncomment `#define EMM42_PC_RETURN 1` in `emm42_servo.h` in config section to enable emm42 servo return message.
* Declare how many motors you want to use by changing `EMM42_MOTOR_N` in `emm42_servo.h`.
* It is advisory to wait a bit (10 ms) after using `emm42_servo_move` and `emm42_servo_uart_turn`, because of UART leftovers.
* One full rotation is 200 steps (in 1 microstep mode).
* After reset servo will zero its position.

## To do list:
* add support for other checksums

## Sources
https://github.com/Macrobase-tech/Emm42