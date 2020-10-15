#include "CM_LIB.H"
#include "CM_I2C.H"
#include "CM_I2C_L.H"
#include "CM_GPA.H"

// Activate Security
//

// CryptoMemory Library Include Files
//xltao add start
uchar ucCM_Encrypt;
uchar ucCM_Authenticate;

uchar ucCM_InsBuff[4];
//xltao add end



// Local function prototypes
static uchar cm_AuthenEncrypt(uchar ucCmd1, uchar ucAddrCi, puchar pucCi, puchar pucG_Sk, puchar pucRandom);
extern int read_from_crypto(unsigned char slaveid, unsigned char* cmdBuffer, unsigned char cmdBytes, unsigned char* readBuffer, unsigned char readBytes );
extern int write_cmd_to_crypto(unsigned char slaveid, unsigned char* pucBuffer, unsigned char ucBytes);

extern void mmdelay(int msec);
// Global Data
uchar ucCM_Ci[8], ucCM_G_Sk[8];
uchar ucCM_Q_Ch[16], ucCM_Ci2[8];

// Activate Security
//
// When called the function:
// ·	reads the current cryptogram (Ci) of the key set, 
// ·	computes the next cryptogram (Ci+1) based on the secret key pucKey (GCi) and the random number selected,
// ·	sends the (Ci+1) and the random number to the CryptoMemory© device, 
// ·	computes (Ci+2) and compares its computed value the new cryptogram of the key set.
// ·	If (Ci+2) matches the new cryptogram of the key set, authentication was successful.
// In addition, if ucEncrypt is TRUE the function:
// ·	computes the new session key (Ci+3) and a challenge, 
// ·	sends the new session key and the challenge to the CryptoMemory© device, 
// ·	If the new session key and the challenge are correctly related, encryption is activated.
//
uchar cm_ActiveSecurity(uchar ucKeySet, puchar pucKey, puchar pucRandom, uchar ucEncrypt)
{
    uchar i;
    uchar ucAddrCi;
    uchar ucReturn;

    // Read Ci for selected key set
    ucAddrCi = CM_Ci + (ucKeySet<<4);              // Ci blocks on 16 byte boundries

    //=====================xltao modify start==========================
    unsigned char cmd_read_c0[3] = /*{0xB6, */{ 0x00, ucAddrCi, 0x08};

    if (read_from_crypto(0xB6, cmd_read_c0, 3, ucCM_Ci, 8 ) < 0)
    {
        return 8;
    }else {
    }

    //if ((ucReturn = cm_ReadConfigZone(ucAddrCi, ucCM_Ci, 8)) != SUCCESS) 
    //	return ucReturn;
    //=========================xltao modify end============================


    // Try to activate authentication
    for (i = 0; i < 8; ++i) ucCM_G_Sk[i] = pucKey[i];
    if ((ucReturn = cm_AuthenEncrypt(ucKeySet, ucAddrCi, ucCM_Ci, ucCM_G_Sk, pucRandom)) != SUCCESS) 
    {
        return ucReturn;
    }
    ucCM_Authenticate = TRUE;

    // If Encryption required, try to activate that too
    if (ucEncrypt) {
        if (pucRandom) pucRandom += 8;
        if ((ucReturn = cm_AuthenEncrypt(ucKeySet+0x10, ucAddrCi, ucCM_Ci, ucCM_G_Sk, pucRandom)) != SUCCESS) 
        {
            return ucReturn;
        }
        ucCM_Encrypt = TRUE;
    }

    // Done
    return SUCCESS;
}

// Common code for both activating authentication and encryption
static uchar cm_AuthenEncrypt(uchar ucCmd1, uchar ucAddrCi, puchar pucCi, puchar pucG_Sk, puchar pucRandom)
{
    uchar i;
    uchar ucReturn;

    // Generate chalange data
    //=====================xltao modify start======================
    if (pucRandom) 
        for (i = 0; i < 8; ++i) 
            ucCM_Q_Ch[i] = pucRandom[i];
    else           
        ;//CM_LOW_LEVEL.RandomGen(ucCM_Q_Ch);
    //=====================xltao modify end========================


    //=====================xltao modify start======================
    for (i = 0; i < 8; ++i) 
        ucCM_Q_Ch[i] = pucRandom[i];
    //=====================xltao modify end========================

    cm_AuthenEncryptCal(pucCi, pucG_Sk, ucCM_Q_Ch, &ucCM_Q_Ch[8]);

    // Send chalange
    ucCM_InsBuff[0] = 0xb8;
    ucCM_InsBuff[1] = ucCmd1;
    ucCM_InsBuff[2] = 0x00;
    ucCM_InsBuff[3] = 0x10;

    //============================xltao modify start==================================
    unsigned char cmd_auth[19] = /*{0xB8, */{ ucCmd1, 0x00, 0x10, 
        ucCM_Q_Ch[0], ucCM_Q_Ch[1], ucCM_Q_Ch[2], ucCM_Q_Ch[3], ucCM_Q_Ch[4], ucCM_Q_Ch[5], ucCM_Q_Ch[6], ucCM_Q_Ch[7],
        ucCM_Q_Ch[8], ucCM_Q_Ch[9], ucCM_Q_Ch[10], ucCM_Q_Ch[11], ucCM_Q_Ch[12], ucCM_Q_Ch[13], ucCM_Q_Ch[14], ucCM_Q_Ch[15]
    };

    //if ((ucReturn = cm_WriteCommand(ucCM_InsBuff, ucCM_Q_Ch, 16)) != SUCCESS) return ucReturn;

    if (write_cmd_to_crypto(0xB8, cmd_auth, 19) < 0)
    {
        return 5;
    }
    //============================xltao modify end====================================

    // Give chips some clocks to do calculations
    //CM_LOW_LEVEL.WaitClock(3); //xltao del the line
    //Sleep(10);                    //xltao add the line
    mmdelay(10);    

    // Verify result
    //=================================xltao modify start================================
    unsigned char cmd_read_c0[3] = /*{0xB6, */{ 0x00, ucAddrCi, 0x08};

    if (read_from_crypto(0xB6, cmd_read_c0, 3, ucCM_Ci2, 8 ) < 0)
    {
        return 5;
    }else {
    }



    //if ((ucReturn = cm_ReadConfigZone(ucAddrCi, ucCM_Ci2, 8)) != SUCCESS) 
    //	return ucReturn;

    for(i=0; i<8; i++) 
        if (pucCi[i]!=ucCM_Ci2[i]) 
        {
            return FAILED2;
        }
    //==================================xltao modify end==================================

    // Done
    return SUCCESS;
}
