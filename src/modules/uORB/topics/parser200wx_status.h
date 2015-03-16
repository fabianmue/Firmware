#ifndef PARSER200WX_STATUS_H
#define PARSER200WX_STATUS_H

#include <stdint.h>
#include "../uORB.h"

//Mask baytes
#define PWS_READ_COG 0x0001
#define PWS_READ_TW  0x0002
#define PWS_READ_AW  0x0004

struct parser200wx_status_s {
    uint64_t timestamp;    /**< Time of the last Pathplanning Update since System Start in Microseconds */
    int32_t byte_read;     /**< Number of byte read from the weather station*/
    uint16_t read_msgs;    /**< Bit mask, shows which msgs have been received */
};


/* register this as object request broker structure */
ORB_DECLARE(parser200wx_status);

#endif // PARSER200WX_STATUS_H
