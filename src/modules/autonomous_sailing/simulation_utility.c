#include "simulation_utility.h"

#if SIMULATION_FLAG == 1

static int32_t temp_cont = 0;

void print_debug_mode(float *pos_p, float *val_p, int numb, struct structs_topics_s *strs_p){

    static int index = 0;
    int tot_time = 30;
    int pos_time = 12;

    strs_p->airspeed.timestamp = hrt_absolute_time();

    if(temp_cont < index * tot_time + pos_time){
       strs_p->airspeed.true_airspeed_m_s = pos_p[index];
    }
    else if(temp_cont < index * tot_time + tot_time){
        strs_p->airspeed.true_airspeed_m_s = val_p[index];
    }
    else{
        index++;
        if(index >= numb){
            index = 0;
            temp_cont = -1;
        }
    }


    temp_cont++;
}




#endif
