#ifndef MPC_TEST_DATA_H
#define MPC_TEST_DATA_H

#include <stdint.h>
#include "settings.h"
#include "guidance_module.h"

#if TEST_MPC == 1

void mt_init_mpc_data(void);

void mt_get_next_yr_y(float x[3]);

bool mt_still_data(void);

float mt_get_correct_rud(void);

#endif

#endif // MPC_TEST_DATA_H
