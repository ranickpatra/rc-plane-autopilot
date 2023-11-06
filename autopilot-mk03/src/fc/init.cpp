#include "./init.h"

#include "io/fin.h"
#include "io/indiactor.h"
#include "io/propeller.h"
#include "io/receiver.h"
#include "flight/imu.h"

systemState_e systemState = SYSTEM_STATE_INITIALISING;

void fc_init() {
    receiver_init();
    indicator_init();
    propeller_init();
    fin_init();

    // stetup IMU
    if (!imu_init()) {
        while (true) {
            indicator_red_blink();
            delay(50);
        }
    }
}