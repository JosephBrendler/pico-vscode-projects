#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"

volatile bool alarm_flag = false;

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    // Put your timeout handler code in here
    alarm_flag = true;
    return 0;
}



int main()
{
    stdio_init_all();

    // Timer example code - This example fires off the callback after 2000ms
    add_alarm_in_us(1, alarm_callback, NULL, false);
    // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

    while (true) {
        if (alarm_flag) {
            printf("Hello, alarmed world!\n");
            alarm_flag = false;
        } else {
            printf("Hello, un-alarmed world!\n");
        }
        sleep_ms(1000);
    }
}
