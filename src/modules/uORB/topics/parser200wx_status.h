#ifndef PARSER200WX_STATUS_H
#define PARSER200WX_STATUS_H

#include <stdint.h>
#include "../uORB.h"

//Mask baytes
#define PWS_READ_GPVTG      0x0001
#define PWS_READ_WIMWD      0x0002
#define PWS_READ_WIVWR      0x0004
#define PWS_READ_GPGGA      0x0008

struct parser200wx_status_s {
    uint64_t timestamp;    /**< Time of the last Pathplanning Update since System Start in Microseconds */
    int32_t byte_read;     /**< Number of byte read from the weather station*/
    uint16_t read_msgs;    /**< Bit mask, shows which msgs have been received */
    float debug1;          /**< Usefull value for debugging */
    uint16_t completed_msgs; /**< Number of full messages read */
};


/* register this as object request broker structure */
ORB_DECLARE(parser200wx_status);

#endif // PARSER200WX_STATUS_H
