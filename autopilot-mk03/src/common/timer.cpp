#include "timer.h"


unsigned long time_count;


void start_time_count() {
    time_count = micros();
}
void end_time_count() {
    time_count = micros() - time_count;
}


unsigned long get_time_count() {
    return time_count;
}