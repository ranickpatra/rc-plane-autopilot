#include "./init.h"

#include "io/fin.h"
#include "io/indiactor.h"
#include "io/propeller.h"
#include "io/receiver.h"

systemState_e systemState = SYSTEM_STATE_INITIALISING;

void fc_init() {
    receiver_init();
    indicator_init();
    propeller_init();
    fin_init();
}