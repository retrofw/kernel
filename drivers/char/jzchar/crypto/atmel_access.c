#define TDEBUG
//#undef TDEBUG
#define FUSE
//#undef FUSE
#define printf
int auth_crypt();
typedef unsigned char BOOL;

extern BOOL I2CSim_CommonWrite(unsigned char ucSlaveID, unsigned char* pucBuffer, unsigned char ucBytes) ;
extern BOOL I2CSim_CommonRead_crypto(unsigned char ucSlaveID, unsigned char* cmdBuffer, unsigned char cmdNumber,unsigned char* pucBuffer, unsigned char ucBytes);

extern void crypt_init();

extern void mmdelay(int msec);
extern unsigned char cm_ActiveSecurity(unsigned char ucKeySet, unsigned char * pucKey, unsigned char * pucRandom, unsigned char ucEncrypt);
int write_cmd_to_crypto(unsigned char slaveid, unsigned char* pucBuffer, unsigned char ucBytes)
{
    int i=0;

    for(i=0; i<5 ; i++)
    {
        //RETAILMSG(1,(TEXT("write cmd_set_user_z0 fail\n")));
        if (I2CSim_CommonWrite(slaveid, pucBuffer, ucBytes)){
            //RETAILMSG(1,(TEXT("xltao write cmd success\n")));
            break;
        }else {
            //RETAILMSG(1,(TEXT("xltao write cmd fail, rewrite\n")));
            if (i < 4){
                continue;
            }else {
                return -1;	
            }
        }
    }

    return 0; 
}

int read_from_crypto(unsigned char slaveid, unsigned char* cmdBuffer, unsigned char cmdBytes, unsigned char* readBuffer, unsigned char readBytes )
{
    int i=0;

    for(i=0; i<5; i++)
    {

        if (I2CSim_CommonRead_crypto(slaveid , cmdBuffer , cmdBytes, readBuffer, readBytes) )
        {
            break;		    
        }else {
            if (i < 4){
                continue;
            }else {
                return -1;
            }
        }

    }

    return 0;
}

/*
 * return value:
 * 0:           OK
 * others:      NG
 */
int fuse()
{
    int i,j;
    unsigned char cmd_set_uz0[3] = {0x03, 0x00, 0x00};  //B4
    unsigned char cmd_write_data[7] = {0x00, 0x00, 0x04, 0x0a, 0x0b, 0x0c, 0x0d};  //B0
    unsigned char cmd_read_data[3] = {0x00, 0x00, 0x04};	//B2	
    unsigned char cmd_read_config32[3] = {0x00, 0x00, 0x20}; //B6
    unsigned char cmd_write_config_mtz[5] = {0x00, 0x0a, 0x02, 0xab, 0xcd};  //B4
    unsigned char cmd_unlock_configz[6] = {0x07, 0x00, 0x03, 0xDD, 0x42, 0x97}; //BA
    unsigned char cmd_write_ci0[10] = {0x00, 0x51, 0x07, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0xa}; //B4
    unsigned char cmd_write_seed0[11] = {0x00, 0x90, 0x08, 0x15, 0x05, 0x12, 0x12, 0x20, 0x08, 0x02, 0x09};	//B4
    unsigned char cmd_read_configall[3] = {0x00, 0x00, 0xF0};  //B6
    unsigned char cmd_write_fuse_FAB[3] = {0x01, 0x06, 0x00};	//B4
    unsigned char cmd_write_fuse_CMA[3] = {0x01, 0x04, 0x00};	//B4
    unsigned char cmd_write_fuse_PER[3] = {0x01, 0x00, 0x00};	//B4
    unsigned char cmd_read_fuse_PER[3] = {0x01, 0x00, 0x01};  //B6
#ifndef FUSE
    //unsigned char cmd_write_UAT[4] = {0x00, 0x18, 0x01, 0x20}; //B4  ultimate AAC
    unsigned char cmd_write_UAT[4] = {0x00, 0x18, 0x01, 0x0b}; //B4  ultimate AAC
    //unsigned char cmd_write_UAT[4] = {0x00, 0x50, 0x01, 0xff}; //B4  ultimate AAC
#endif

    unsigned char read_buf[256] = {0};


    crypt_init();

    if(write_cmd_to_crypto(0xB4, cmd_set_uz0, 3) < 0)
    {
        printf("set uz0 error!\n");
        return -11;
    }

    mmdelay(1);

    if(write_cmd_to_crypto(0xB0, cmd_write_data, 7) < 0)
    {
        printf("write uz0 error!\n");
        return -12;
    }

    mmdelay(1);

    if(write_cmd_to_crypto(0xB4, cmd_set_uz0, 3) < 0)
    {
        printf("set uz0 error!\n");
        return -13;
    }

    mmdelay(1);

    if(read_from_crypto(0xB2, cmd_read_data, 3, read_buf, 0x04) < 0)
    {
        printf("read uz0 error!\n");
        return -21;
    }


    //set the secu_level of uz0

    mmdelay(10);

    if(read_from_crypto(0xB6, cmd_read_config32, 3, read_buf, 0x20) < 0)
    {
        printf("read config32 error!\n");
        return -22;
    }

    if(write_cmd_to_crypto(0xB4, cmd_write_config_mtz, 5) < 0)
    {
        printf("write config zone error!\n");
        return -14;
    }

    mmdelay(1);



    mmdelay(1);

    if(read_from_crypto(0xB6, cmd_read_config32, 3, read_buf, 0x20) < 0)
    {
        printf("read config32 error!\n");
        return -23;
    }

    //unlock the config zone
    if(write_cmd_to_crypto(0xBA, cmd_unlock_configz, 6) < 0)
    {
        printf("unlock config zone error!\n");
        return -15;
    }

    mmdelay(5);


#ifndef FUSE
#if 0
    if(write_cmd_to_crypto(0xB4, cmd_write_UAT, 4) < 0)
    {
        printf("write config zone error!\n");
        return -31;
    }
#endif
#endif

    //write Ci of z0
    if(write_cmd_to_crypto(0xB4, cmd_write_ci0, 10) < 0)
    {
        printf("wirte ci0 error!\n");		
        return -16;
    }

    mmdelay(5);
    //write Seed of z0
    if(write_cmd_to_crypto(0xB4, cmd_write_seed0, 11) < 0)
    {
        printf("wirte ci0 error!\n");		
        return -17;
    }

    mmdelay(1);
    //read entire config zone
    if(read_from_crypto(0xB6, cmd_read_configall, 3, read_buf, 0xFF) < 0)
    {
        printf("read config zone error!\n");
        return -24;
    }

    mmdelay(1);
    //print config zone to verify
#if 0
    if ((read_buf[9*16+0] == 0x11) && (read_buf[9*16+1] = 0x22) && (read_buf[9*16+2] == 0x33) && (read_buf[9*16+3] == 0x44))
    {
        printf("Auth success\n");
    }
    else
    {
        printf("Auth error\n");
        while(1);
    }
#endif

#ifdef FUSE 
    //write the fuses
    if(write_cmd_to_crypto(0xB4, cmd_write_fuse_FAB, 3) < 0)
    {
        printf("wirte FAB error!\n");	
        return -32;
    }


    if(write_cmd_to_crypto(0xB4, cmd_write_fuse_CMA, 3) < 0)
    {
        printf("wirte CMA error!\n");		
        return -33;
    }


    if(write_cmd_to_crypto(0xB4, cmd_write_fuse_PER, 3) < 0)
    {
        printf("wirte PER error!\n");		
        return -34;
    }


#endif
    //read the fuses for sure
    if(read_from_crypto(0xB6, cmd_read_fuse_PER, 3, read_buf, 0x01) < 0)
    {
        printf("read  fuse PER error!\n");
        return -25;
    }

    mmdelay(10);
#if 0
    mmdelay(1000);
    if (auth() < 0)
    {
        return -41;
    }
#endif

    return 0;
}	

void uudelay(unsigned int usec)
{
	unsigned int i = usec * (360000000 / 2000000);
	__asm__ __volatile__ (
			"\t.set noreorder\n"
			"1:\n\t"
			"bne\t%0, $0, 1b\n\t"
			"addi\t%0, %0, -1\n\t"
			".set reorder\n"
			: "=r" (i)
			: "0" (i)
		);
}

void mmdelay(int msec)
{	
	int i;
	for(i=0; i<msec; i++)
	uudelay(1000);
}	

int auth_crypt(){
    unsigned char ucG[8];

    //unsigned char cmd_write_seed0[11] = {0x00, 0x90, 0x08, 0x15, 0x05, 0x12, 0x12, 0x20, 0x08, 0x02, 0x09};	//B4
    ucG[0] = 0x15;
    ucG[1] = 0x05;
    ucG[2] = 0x12;
    ucG[3] = 0x12;
    ucG[4] = 0x20;
    ucG[5] = 0x08;
    ucG[6] = 0x02;
    ucG[7] = 0x09;


    //unsigned char random[8] = {0x1d, 0xff, 0x32, 0x78, 0x12, 0x34, 0x56, 0x78};
    unsigned char random[8] = {0xdd, 0xff, 0x32, 0x78, 0x12, 0x34, 0x56, 0x78};
    unsigned char rtn;


#if 0	
    //===================xltao add start============================
    unsigned char cmd_write_uz0_secure[5] = /*{0xB4, */{ 0x00, 0x20, 0x02, 0xDF, 0x3F};

    if (write_cmd_to_crypto(0xB8, cmd_write_uz0_secure, 0) < 0)
    {
        printf("maddrone write z0 error!\n");
        return ;
    }else {
        printf("maddrone read z0 success!\n");
    }
#endif	
    //==================xltao add end==============================

    if ((rtn = cm_ActiveSecurity(0, ucG, random, 0)) != 0)
    {
        return -1;
        //while(1);
    }else {
    }
    return 0;

}

EXPORT_SYMBOL(auth_crypt);
