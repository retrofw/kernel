#ifndef __DVB_H__
#define __DVB_H__

#include "nmibustypes.h"
#include "nmidbg.h"
#include "nmidvb.h"
#include "nmitypes.h"
#include "nmidtv.h"
#include "nmibus.h"
#include "nmidrv.h"
#include "nmicfg.h"

#include "i2c.h"

typedef struct
{
        unsigned int            frequency;
        int                                     bandwidth;
        int                                     force;
        int                                     softassist;
        int                                     initok;
        int                                     mode;
        int                                     guard;
        int                                     mod;
        int                                     alpha;
        int                                     rate;
} DVBCORE;


typedef struct
{
        int                                     mode;                                   /* 0: DVB-H, 1: DVB-T */
        union {
                struct {
                        int                                             npid;                   /* number of PID to enable */

                        unsigned short  pid[8];                 /* PID */
                        unsigned char           fec[8];                 /* FEC row */
                        int                                             type[8];                /* 0: inactive, 1: viewing, 2:
 favorite */
                } h;
                struct {
                        int                                             enfilt;                 /* 0: no PID filtering, 1: ena
ble PID filtering */
                        int                                             btbl;                           /* 0: filters are for
data pids, 1: filters are for table pids */
                        int                                             npid;                   /* number of PID to filter */
                        unsigned short  pid[16];                /* filter PID */
                } t;
        } u;
} DVBLNK;



#endif
