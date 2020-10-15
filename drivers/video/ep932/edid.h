/******************************************************************************\

          (c) Copyright Explore Semiconductor, Inc. Limited 2005
                           ALL RIGHTS RESERVED 

--------------------------------------------------------------------------------

  File        :  Edid.h 

  Description :  Head file of Edid IO Interface 

\******************************************************************************/

#ifndef EDID_H
#define EDID_H

#define EDID_BLOCK_SIZE  128



// Structure Definitions

extern unsigned char edid_gethdmicap(unsigned char *ptarget);
extern unsigned char edid_getpcmfreqcap(unsigned char *ptarget);
extern unsigned char edid_getpcmchannelcap(unsigned char *ptarget);
extern unsigned char edid_getdatablockaddr(unsigned char *ptarget, unsigned char tag);

#endif // EDID_H


