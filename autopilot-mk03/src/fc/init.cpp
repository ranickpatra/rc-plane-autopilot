#include "./init.h"

#include "flight/imu.h"
#include "global.h"
#include "io/fin.h"
#include "io/indiactor.h"
#include "io/propeller.h"
#include "io/receiver.h"
#include "states.h"

systemState_e system_state = SYSTEM_STATE_INITIALISING;

void fc_init() {
    receiver_init();
    indicator_init();
    propeller_init();
    fin_init();

    if (!imu_init()) {
        while (true) {
            indicator_red_blink();
            delay(50);
        }
    }
}