#include "./init.h"

#include "flight/imu.h"
#include "global.h"
#include "io/fin.h"
#include "io/indiactor.h"
#include "io/propeller.h"
#include "io/receiver.h"
#include "flight/pid.h"
#include "states.h"

systemState_e states_system_state = SYSTEM_STATE_INITIALISING;

void init_fc_init() {
    receiver_init();
    indicator_init();
    propeller_init();
    fin_init();
    pid_init();

    if (!imu_init()) {
        while (true) {
            indicator_red_blink();
            delay(50);
        }
    }
}