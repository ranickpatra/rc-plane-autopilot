#include "timer.h"


unsigned long timer_time_count;


void timer_start_time_count() {
    timer_time_count = micros();
}
void timer_end_time_count() {
    timer_time_count = micros() - timer_time_count;
}


unsigned long timer_get_time_count() {
    return timer_time_count;
}