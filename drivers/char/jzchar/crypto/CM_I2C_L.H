// CryptoMemory I2C Very Low Level Header File
//
// This library is used in all of the low level hardware interface for I2C communications.
//
// Revision Date      Changes
// -------- --------- -----------------------------------------------------------------
//    0.1   07 Nov 04 First Release

// Protect Library against multiple inclusion
#ifndef CM_I2C_L_H
#define CM_I2C_L_H

// -------------------------------------------------------------------------------------------------
// Other includes required by this header file
// -------------------------------------------------------------------------------------------------

#include "CM_LIB.H"
#include "CM_I2C.H"

// Constants used in low level functions
// Power on clocks (spec call for 5, but California Card uses 15)
#define CM_PWRON_CLKS (15)

// Mid-Level Functions
uchar cm_ReadCommand(puchar pucInsBuff, puchar pucRetVal, uchar ucLen);
uchar cm_WriteCommand(puchar pucInsBuff, puchar pucSendVal, uchar ucLen);

// Functions in CM_I2C.C used internally by other low level functions
void cm_Clockhigh(void);
void cm_Clocklow(void);
void cm_ClockCycle(void);
void cm_ClockCycles(uchar ucCount);
void cm_Start(void);
void cm_Stop(void);
uchar cm_Write(uchar ucData);
void cm_Ack(void);
void cm_N_Ack(void);
uchar cm_Read(void);
void cm_WaitClock(uchar loop);
uchar cm_SendCommand(puchar pucInsBuff);
uchar cm_ReceiveRet(puchar pucRecBuf, uchar ucLen);
uchar cm_SendDat(puchar pucSendBuf, uchar ucLen);
void cm_RandomGen(puchar pucRanddat);

// functions in CM_LOW.C used internally by other low level functions
void cm_Delay(uchar ucDelay);

// end of multiple inclusion protection
#endif
