
#define LOGE  printk
#define LOGD  printk

#include <linux/delay.h>

#include "data.h"
#include "isp.h"

#define SIMULATE_I2C  

//we have 2 suite simulate i2c interface if burn new fw ,define i2c-1
//#define i2c-1

extern unsigned int i2c_addr;
extern unsigned int i2c_clk;

/* I2C APP */
extern int i2c_write_16(unsigned char device, unsigned char *buf, unsigned short address, int count);
extern int i2c_read_16(unsigned char device, unsigned char *buf, unsigned short address, int count);


extern void sensor_write_reg(unsigned char reg, unsigned char val);
extern int sensor_write_reg16(unsigned short reg, unsigned char val);
extern unsigned char sensor_read_reg(unsigned char reg);
extern unsigned char sensor_read_reg16(unsigned short reg);

extern void I2CSim_Write8(unsigned char ucSlaveID, char address ,char value);
extern unsigned char I2CSim_Read8(unsigned char ucSlaveID, char address);

extern unsigned short I2C_ReadControl(unsigned char IICID, unsigned short regaddr);
extern unsigned short I2C_WriteControl(unsigned char IICID, unsigned short regaddr, unsigned short data);


//#define INIT_IIC_BURST_MODE   // Jacky for all reg burst mode	

faceInfo _faceInfo[10];

#ifdef HISENSE_ZOOM_TEST
CAM_ZOOM_T g_curDigitalZoom = CAM_ZOOM_480_360;
#else
unsigned char g_curDigitalZoom = 0;
#endif

void sensor_set_addr(int addr)
{
#ifndef SIMULATE_I2C 
	i2c_addr = addr;
#endif
}

// Following System IIC you need chang to you system IIC
/*********************************************************************
*Name			:	System_IICWrite
*Description	:		
*Param			:	IIC ID Address and data
*return			:	none
*Author			:	
*Remark			:	You have to use instead of System's function
*Log			:	
**********************************************************************/
void System_IICWrite( ClUint_8 id, ClUint_8 addr,  ClUint_8 data)
{
/*#if defined(ISP3_EVB_ENABLE)||defined(ISP2_EVB_ENABLE)  

	SI2CID = id;
	printk("%s : 0x%x\n",__func__, ((SI2CID & 0x7f) << 1));
	IIC_WRITE(addr, data);
#else
#endif */
#ifdef SIMULATE_I2C
        I2CSim_Write8(id, addr, data);
#else
        sensor_set_addr(id);
        sensor_write_reg(addr,data);

#endif
}

/*********************************************************************
*Name			:	System_IICRead
*Description	:	
*Param			:	IIC addr
*return			:	iic data
*Author			:	
*Remark			:	You have to use instead of System's function
*Log			:	
**********************************************************************/
ClUint_16 System_IICRead(ClUint_16 addr)
{
	ClUint_16 data = 0;
	ClUint_32 delay_cnt = 0;

/*#if defined(ISP3_EVB_ENABLE)||defined(ISP2_EVB_ENABLE)
	System_IICWrite(ISP_IICID_A, 0x01, addr>>8);				//3 2. IIC Addr = 0x01

	SI2CID = ISP_IICID_B;
	data = IIC_READ((ClUint_16)(addr&0xff));		//3           5. IIC Addr = low 8 bit of addr value. & Read Data with IIC ID = 0x80
	printk("%s :: 0x%x : 0x%x\n",__func__, addr, data);
#endif*/
#ifdef SIMULATE_I2C
#ifdef i2c-1
	System_IICWrite(ISP_IICID_A, 0x01, addr>>8);
	data = I2CSim_Read8(ISP_IICID_B, (addr&0xff));
#else
	I2C_WriteControl(ISP_IICID_A, 0x01, addr>>8);
	data = I2C_ReadControl(ISP_IICID_B, (addr&0xff));
#endif
#else
	//3 Please porting your system IIC read function here
	System_IICWrite(ISP_IICID_A, 0x01, addr>>8);				//3 2. IIC Addr = 0x01
	sensor_set_addr(ISP_IICID_B);
	data = sensor_read_reg((unsigned char)(addr&0xff));

#endif

	return data; 
}

/*********************************************************************
*Name			:	CoreISP3_I2C_Read
*Description	:		Read Data from register of addr
*Param			:	addr = read register
*return			:	Register value
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
ClUint_16 CoreISP3_I2C_Read( ClUint_16 addr )
{
	ClUint_16 data = 0;

	data = System_IICRead(addr);

	return data; 
}

#ifdef INIT_IIC_BURST_MODE

void CoreISP3_I2C_Read_Bulk( ClUint_16 addr, ClUint_8 data )
{
	ClUint_16 data = 0;

#if defined(ISP3_EVB_ENABLE)||defined(ISP2_EVB_ENABLE)
	SI2CID = ISP_IICID_B;
	data = IIC_READ((ClUint_16)(addr&0xff));		//3           5. IIC Addr = low 8 bit of addr value. & Read Data with IIC ID = 0x80
#else   // Please porting your iic read function
#endif

	return data; 
}
#endif

/*********************************************************************
*Name			:	CoreISP3_I2C_Write
*Description	:		Write Data
*Param			:	addr = Addr of Register
					data = Setting value
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
int CoreISP3_I2C_Write( ClUint_16 addr, ClUint_8 data )
{
#ifdef SIMULATE_I2C
#ifdef i2c-1
	System_IICWrite(ISP_IICID_A, 0x01, addr>>8);
	System_IICWrite(ISP_IICID_B, addr&0xff, data);
	mdelay(5);
#else
	
	I2C_WriteControl(ISP_IICID_A, 0x01, addr>>8);
	I2C_WriteControl(ISP_IICID_B, addr&0xff, data);
#endif
#else 
	I2C_WriteControl(ISP_IICID_A, 0x01, addr>>8);
        I2C_WriteControl(ISP_IICID_B, addr&0xff, data);


#endif
	
//	if(addr!=0xe660)
//	printk("System_IIC_Write_check:write(0x%x)=0x%x,read=0x%x\n",addr,data,System_IICRead(addr));
	return 0;
}

void CoreISP3_I2C_Write_Bulk( ClUint_16 addr, ClUint_8 data )
{
	//System_IICWrite(ISP_IICID_B, addr&0xff, data);
	I2C_WriteControl(ISP_IICID_B, addr&0xff, data);;
}


void CoreISP3_I2C_Partial_Write( ClUint_16 Addr, ClUint_16 HighBit, ClUint_16 LowBit, ClUint_8 Data )
{
	ClUint_8 BitMask = 0xff;
	ClUint_8 ReadData;

	BitMask = BitMask<<(7-HighBit);
	BitMask = BitMask>>(7-HighBit+LowBit);
	BitMask = BitMask<<LowBit; 
	BitMask = ~BitMask;

	ReadData = CoreISP3_I2C_Read(Addr);
	if(ReadData < 0)
		return;

	ReadData = ReadData&BitMask;
	ReadData = ReadData |Data<<LowBit;

	CoreISP3_I2C_Write(Addr, ReadData);
	printk( ">>CoreISP3_I2C_Partial_Write>>Addr 0x%x = 0x%x \n ", Addr, CoreISP3_I2C_Read(Addr));
}



/*********************************************************************
*Name			:	CoreISP3_Send_Command
*Description	:	Send "ISP3_COMMAND" to ISP3
*Param			:	Cmd_Type	ISP COMMAND
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_Send_Command(unsigned char  Cmd_Type)// enISP3_CMD_TYPE Cmd_Type )
{
	ClUint_16 retval = 1, i = 0;
	printk("Send Command to ISP MCU [Command Type : 0x%x]\n", Cmd_Type);

	CoreISP3_I2C_Write(ISP3_COMMAND, Cmd_Type);
	WaitTime_us(2*100);    // 100ms  // Jacky add this
	while(retval)
	{
		retval = CoreISP3_I2C_Read(ISP3_COMMAND);
		//for(i=0;i<0xffff;i++);  // must add this delay tiem... or 100ms delay
		WaitTime_us(2*100);    // 100ms  // Jacky for test
		#if 1
		i++;
		if(i>50)
			{
			printk( "send isp cmd[0x%x] fail value 0x%x \n", Cmd_Type, retval);
			break;
			}
		#endif
		printk( "@@0x%x\n",retval);
	}
}
/*********************************************************************
*Name			:	CoreISP3_SetClock
*Description	:	ISP3 Configuration(Set PLL of ISP3 & Initialize)
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetClock( void )
{
	//3 PLL OFF


	//3 FOUT = {M x FIN} / {P x 2^S} : FOUT is the MCLK
	//3 (24MHz x 54) / (6 x 2^2) = 54MHz           24M x 24 /(5x2^2) = 
	// 24MHz => 60MHz      (26MHz for YuHua)
	CoreISP3_I2C_Write(0xe062, 0x06);  //P Div
	CoreISP3_I2C_Write(0xe061, 0x18);  //M Div
	CoreISP3_I2C_Write(0xe063, 0x02);  //S  Div
	
	CoreISP3_I2C_Write(0xe060, 0x03);	//4 PLL Register
	printk( "0xe062[0x%x], 0xe061[0x%x], 0xe063[0x%x]\n", CoreISP3_I2C_Read(0xe062) , CoreISP3_I2C_Read(0xe061), CoreISP3_I2C_Read(0xe063));

/*
[6:6] rw 0x00 CIS1SRSTMODE When this bit is set to “1”, S1_RST and S1_PWDN will be 
controlled by CIS1RESET and CIS1PWDN. 
[5:5] rw 0x00 CIS1SRSTBIT This bit controls S1_RST pin state when CIS1REGCTL bit    0xc0   1100 0000        e0  1110
is set to “1” 
[4:4] rw 0x00 CIS1SPWDNBIT This bit controls S1_PWDN pin state when CIS1REGCTL bit 
is set to “1” 
*/
	//3 Enable parallel sensor port 1 interface
	//3 Sensor Reset
	CoreISP3_I2C_Write(0xe010, 0xf0);	//4 [7] : Sensor Port 1 I/F Enable
										//4 [6] : S1_RST & S1_PWDN control by CIS1RESET & CIS1PWDN
										//4 [5] : S1_RST = 1

	//3 Delay
	WaitTime_us(2*10);				//4 10ms

	CoreISP3_I2C_Write(0xe010, 0xd0);   //4 [7] : Sensor Port 1 I/F Enable            // d   1101
										//4 [6] : S1_RST & S1_PWDN control by CIS1RESET & CIS1PWDN
										//4 [5] : S1_RST = 0
										
	WaitTime_us(2*100);				//4 50ms

	CoreISP3_I2C_Write(0xe010, 0xf0);	//4 [7] : Sensor Port 1 I/F Enable
										//4 [6] : S1_RST & S1_PWDN control by CIS1RESET & CIS1PWDN
										//4 [5] : S1_RST = 1


//	CoreISP3_I2C_Write(0xe0a2, 0x19);  //Flash mode  Jacky for test

	// Read default value 0xe013[0x21], 0xe015[0x11], 0xe011[0x1]  0xe012[0x1], 0xe014[0x12] 
	printk( "0xe013[0x%x], 0xe015[0x%x], 0xe011[0x%x]\n", CoreISP3_I2C_Read(0xe013) , CoreISP3_I2C_Read(0xe015), CoreISP3_I2C_Read(0xe011));
	printk( "0xe012[0x%x], 0xe014[0x%x] \n", CoreISP3_I2C_Read(0xe012) , CoreISP3_I2C_Read(0xe014));
	//3 Div from GCLK : MCLK, FCLK, JMCLK
	//CoreISP3_I2C_Write(0xe013, 0x21);	//4 [7:4] Cis1IntD(SCLK Divider Value)
										//4 ISP System Clock : FOUT x 1/2 = 27MHz             
										//4 [3:0] Cis1IntD(MCLK Divider Value)
										//4 ISP ISP Memory Clock : FOUT x 1 = 54MHz
	
	//CoreISP3_I2C_Write(0xe015, 0x11);    //4[7:4] FClkDiv(FCLK) => 54MHz, [2:0] JMClkDiv(JMCLK) => 54MHz
	
	//0xe012	[2:0] rw 0x01 Cis1IntC SMCLK divider value 
	//			(1: x1, 2:x1/2, 4:x1/4, 8:x1/8, 4:x1/16)
	//3 Div from MCLK : SMCLK, 8051CLK, SCLK, JCLK

	CoreISP3_I2C_Write(0xe012, 0x01);    //4[2:0] Cis1ClkDiv(S1_MCLK) => 27MHz
	mdelay(100);

	CoreISP3_I2C_Write(0xe014, 0x12);    //4[7:4] i8051_ClkDiv => 27MHz, [3:0] JClkDiv(JCLK) => 27MHz
	

	/* CCLK Setting */
	//CoreISP3_I2C_Write(0xe011, 0x00);    //4[5:4] '0' =>S1_PCLK, '1' => S1_MCLK, '2' => SCLK   
	WaitTime_us(2*10);    // 10ms    
	
	/* PLL ON */
	CoreISP3_I2C_Write(0xe060, 0x00);
	//WaitTime_us(2*500);    // 500ms     Jacky delete this for test ok

}


/*********************************************************************
*Name			:	CoreISP3_LSC_TableDownLoad
*Description	:	Downloading Lens Shading Data
*Param			:	none
*return			:	Cl_True : Scuuess, Cl_False : Fail
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
#ifndef ISP3_MCU_LSC
Cl_Bool CoreISP3_LSC_TableDownLoad( void )
{
	ClUint_32	data_cnt;
	ClUint_16	reg_data;
	ClUint_32	i;

	printk("[CoreISP3_LSC_TableDownLoad]LSC Binary Size : :%d\n", LSC_SIZE);

#ifdef INIT_IIC_BURST_MODE
	System_IICWrite(ISP_IICID_A, 0x01, 0xe0);
	CoreISP3_I2C_Write_Bulk(IspFenA, 0x00);
	
	System_IICWrite(ISP_IICID_A, 0x01, 0xe2);
	CoreISP3_I2C_Write(ShdLMDsizeH, LSC_SIZE/256);
	CoreISP3_I2C_Write(ShdLMDsizeL, LSC_SIZE%256);
#else
	/* -LSC(Lens Shading Compension) Function disable.*/
	CoreISP3_I2C_Write(IspFenA, 0x00);

	// -LSC Size Setting
	CoreISP3_I2C_Write(ShdLMDsizeH, LSC_SIZE/256);
	CoreISP3_I2C_Write(ShdLMDsizeL, LSC_SIZE%256);

	printk(">>ISP2_REG>>0xe2e2==0x%x[0x%x]\n",CoreISP3_I2C_Read(ShdLMDsizeH), LSC_SIZE/256);
	printk(">>ISP2_REG>>0xe2e3==0x%x[0x%x]\n",CoreISP3_I2C_Read(ShdLMDsizeL), LSC_SIZE%256);
	if(CoreISP3_I2C_Read(ShdLMDsizeH)!= LSC_SIZE/256)
		{
		printk("IIC_Write Read error...\n");
		return ClFalse;
		}
	// -Download LSC file
	printk("LSC Binary Downloading.....>>>>>>>>>>.\n");
	printk("%s<CoreISP3_I2C_Write_Bulk> : 0x%x\n",__func__, ((SI2CID & 0x7f) << 1));
#endif

	for(data_cnt=0; data_cnt<LSC_SIZE; data_cnt++)
	{
		//4 0x80
		CoreISP3_I2C_Write_Bulk(ShdLMData, LSC_INIT_TABLE[data_cnt]);
	}

	printk("LSC Binary Download End!!\n");

	CoreISP3_I2C_Write(IspFenA, 0x01);   // -LSC Function enable.

#if 0	//3 Verify LSC Data
	printk("LSC Verify START @@@@@@@@@@@@@@@@@@@@@\n");
	for( i=0; i<LSC_SIZE; i++ ) 
	{
		//3 Set Address
		CoreISP3_I2C_Write(0xe2e0,	i>>8);		//4 Set Upper Address
		CoreISP3_I2C_Write(0xe2e1,	i & 0xFF);	//4 Set Lower Addreaa
		reg_data = CoreISP3_I2C_Read(0xe2e4);	//4 Read Data of Address
		reg_data = CoreISP3_I2C_Read(0xe2e4);	//4 Read Data of Address

		if (reg_data != LSC_INIT_TABLE[i])
		{
			printk("LSC Verify ERROR LSC_INIT_TABLE[%d] = 0x%x : 0x%x !!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", i, reg_data, LSC_INIT_TABLE[i]);
			return Cl_False;
		}

	}
	printk( "LSC Verify END @@@@@@@@@@@@@@@@@@@@@\n");
#endif

	return Cl_True;
}
#endif
/*********************************************************************
*Name			:	ISP3_FlashromFormat
*Description	:	Format Flash of ISP3
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void ISP3_FlashromFormat(void)
{
#ifdef INIT_IIC_BURST_MODE
	System_IICWrite(ISP_IICID_A, 0x01, 0xe0);
	CoreISP3_I2C_Write_Bulk(0xE070, 0xFD);	// MCU hold
	CoreISP3_I2C_Write_Bulk(0xE0C4, 0x10);
	CoreISP3_I2C_Write_Bulk(0xE0A2, 0x19);	// Flashrom mode
	CoreISP3_I2C_Write_Bulk(0xE0A1, 0x01);	// Format
#else	// Flashrom Format
	CoreISP3_I2C_Write(0xE070, 0xFD);	// MCU hold
	CoreISP3_I2C_Write(0xE0C4, 0x10);
	CoreISP3_I2C_Write(0xE0A2, 0x19);	// Flashrom mode
	CoreISP3_I2C_Write(0xE0A1, 0x01);	// Format
#endif	
}
/*********************************************************************
*Name			:	ISP3_FlashromBlockErase
*Description	:	Erase Flash of ISP3 Block
*Param			:	eraseStartAddress	Start Address of Flash
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void ISP3_FlashromBlockErase(int eraseStartAddress)
{
	// Flash Block Erase
	CoreISP3_I2C_Write(0xE070, 0xFD);	// MCU Hold
	CoreISP3_I2C_Write(0xE0BA, eraseStartAddress>>8);
	CoreISP3_I2C_Write(0xE0BB, eraseStartAddress&0xFF);
	CoreISP3_I2C_Write(0xE0C4, 0x30);
	CoreISP3_I2C_Write(0xE0A1, 0x01);
}

#ifdef IIC_BURST_MODE
void ISP3_FlashromBurstWrite(int writeStartAddress, ClUint_8* data, int length, void (*callbackFunc)(ClUint_8 *data, int n, int nCount))
{
	int i;
	int writeEndAddress = writeStartAddress+length-1;
	ClUint_16 val;

	val = CoreISP3_I2C_Read(0xE037);
	printk("write burst\n");

#ifdef INIT_IIC_BURST_MODE
	System_IICWrite(ISP_IICID_A, 0x01, 0xe0);
	CoreISP3_I2C_Write_Bulk(0xE037, val&~0x80);	// Address Auto Inc Off
	
	// Flash Write
	CoreISP3_I2C_Write_Bulk(0xE070, 0xFD);
	CoreISP3_I2C_Write_Bulk(0xE0A3, writeStartAddress>>8);
	CoreISP3_I2C_Write_Bulk(0xE0A4, writeStartAddress&0xFF);
	CoreISP3_I2C_Write_Bulk(0xE0A6, writeEndAddress>>8);
	CoreISP3_I2C_Write_Bulk(0xE0A7, writeEndAddress&0xFF);
	CoreISP3_I2C_Write_Bulk(0xE0A2, 0x29);
	CoreISP3_I2C_Write_Bulk(0xE0A2, 0x19);
	WaitTime_us(2*50);
	for(i=0; i<length; i++)
	{
		CoreISP3_I2C_Write_Bulk(0xE0A5, data[i]);
		if(callbackFunc != Cl_Null)
		{
			callbackFunc(data, i, length);
		}
		else
		{
			//Sleep(2);
		}
	}

	CoreISP3_I2C_Write(0xE037, val);
#else	
	CoreISP3_I2C_Write(0xE037, val&~0x80);	// Address Auto Inc Off
	
	// Flash Write
	CoreISP3_I2C_Write(0xE070, 0xFD);
	CoreISP3_I2C_Write(0xE0A3, writeStartAddress>>8);
	CoreISP3_I2C_Write(0xE0A4, writeStartAddress&0xFF);
	CoreISP3_I2C_Write(0xE0A6, writeEndAddress>>8);
	CoreISP3_I2C_Write(0xE0A7, writeEndAddress&0xFF);
	CoreISP3_I2C_Write(0xE0A2, 0x29);
	CoreISP3_I2C_Write(0xE0A2, 0x19);
	WaitTime_us(2*50);
	for(i=0; i<length; i++)
	{
		CoreISP3_I2C_Write_Bulk(0xE0A5, data[i]);
		if(callbackFunc != Cl_Null)
		{
			callbackFunc(data, i, length);
		}
		else
		{
			//Sleep(2);
			//printk("%d,0x%x\n",i,data[i]);
			//WaitTime_us(10);
		}
	}

	CoreISP3_I2C_Write(0xE037, val);
#endif	
}

#else

/*********************************************************************
*Name			:	ISP3_FlashromWrite
*Description	:	Write to Flash of ISP3
*Param			:	writeStartAddress	Start address of Flash
					data				Point of Data
					length				Data Size
					callbackFunc		Call back function
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void ISP3_FlashromWrite(int writeStartAddress, ClUint_8* data, int length, void (*callbackFunc)(ClUint_8 *data, int n, int nCount))
{
	int i;
	int writeEndAddress = writeStartAddress+length-1;
	// Flash Write
	printk("ISP3_FLASH-WRITE\n");
	CoreISP3_I2C_Write(0xE070, 0xFD);
	CoreISP3_I2C_Write(0xE0A3, writeStartAddress>>8);
	CoreISP3_I2C_Write(0xE0A4, writeStartAddress&0xFF);
	CoreISP3_I2C_Write(0xE0A6, writeEndAddress>>8);
	CoreISP3_I2C_Write(0xE0A7, writeEndAddress&0xFF);
	CoreISP3_I2C_Write(0xE0A2, 0x29);
	CoreISP3_I2C_Write(0xE0A2, 0x19);
	WaitTime_us(2*50);				//4 50 ms

	for(i=0; i<length; i++)
	{	printk("%d,0x%x\n",i,data[i]);
		CoreISP3_I2C_Write(0xE0A5, data[i]);
		if(callbackFunc != Cl_Null)
		{
			callbackFunc(data, i, length);
		}
		else
		{
			//			Sleep(2);
		}
	}
}
#endif //4 IIC_BURST_MODE

/*********************************************************************
*Name			:	ISP3_FlashromRead
*Description	:	Read from Flash of ISP3
*Param			:	readStartAddress		Start address of Flash
					data					Save Reading data
					length					Read size
					callbackFunc			Call back function
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void ISP3_FlashromRead(int readStartAddress, ClUint_8 *data, int length, void (*callbackFunc)(ClUint_8 *data, int n, int nCount))
{
	int i;
	// Dump
	CoreISP3_I2C_Write(0xE070, 0xFD);
	CoreISP3_I2C_Write(0xE0A2, 0x31);

	for(i=0; i<length; i++)
	{
		int addr = readStartAddress+i;
		CoreISP3_I2C_Write(0xE0A3, addr>>8);	// Address High
		CoreISP3_I2C_Write(0xE0A4, addr&0xFF);	// Address Low
		data[i] = CoreISP3_I2C_Read(0xE0A5);
		//		data[i] = clIIC_Read(0xE0A5);
		if(callbackFunc != Cl_Null)
		{
			callbackFunc(data, i, length);
		}
		else
		{
			//			Sleep(2);
		}
	}
}


ClUint_8 CoreISP3_Code_Verification( ClUint_8 * pVerifyData, ClUint_32 VerifyDataSize, enIsp3DownloadMode ModeParam )
{
	ClUint_8 RetVal = 0;
	ClUint_32 data_cnt, reg_cnt, RegData;

	/* MCU Reset */
	CoreISP3_I2C_Write(MCURST, 0x05);
	WaitTime_us(2*100);    // 100ms
	CoreISP3_I2C_Write(MCURST, 0x04);
	WaitTime_us(2*100);    // 100ms

	if(ModeParam == enDownloadMode_StackedMem)
	{
		CoreISP3_I2C_Write(ShdCenY, 0x31);
	}
	else
	{
		CoreISP3_I2C_Write(ShdCenY, 0xa0);
	}
	reg_cnt = 0;
	for(data_cnt=0; data_cnt<VerifyDataSize+1; data_cnt++)
	{
		CoreISP3_I2C_Write(ShdCnvRtoH, data_cnt/256);
		CoreISP3_I2C_Write(ShdCnvRtoL, data_cnt%256);
		RegData = CoreISP3_I2C_Read(FlashData);
		if(data_cnt>0) 
		{
			if(RegData != pVerifyData[reg_cnt])
			{
				reg_cnt++;
				printk( "MCU Bin Verify ERROR pInitialData[%d] = 0x%x : 0x%x @@@@@@@@@@@@@@@@@@@@@\n", data_cnt, RegData, pVerifyData[reg_cnt]);
				return 1;
			}
		}
	}
	return RetVal;
}

/*********************************************************************
*Name			:	CoreISP3_MCU_CodeDownload
*Description	:	Downloading ISP3 LSC & Main Bin Data
*Param			:	pInitialData	Pointer of MCU Data
					InitialDataSize	Length of MCU Data
					ModeParam		enDownloadMode_StackedMem		Write to Flash
									enDownloadMode_SkipedMCUBin		Read from Flash
									enDownloadMode_CodeRAM			Write to RAM
*return			:	Cl_True : Scuuess, Cl_False : Fail
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
ClUint_8 CoreISP3_MCU_CodeDownload( ClUint_8 * pInitialData, ClUint_32 InitialDataSize, enIsp3DownloadMode ModeParam )
{
	ClUint_8 RetVal = 0;
	ClUint_16 reg_data;
	ClUint_32 i;
	int earseAddr=0;

	printk( "[CoreISP3_MCU_CodeDownload]MCU Binary Size : :%d, ModeParam : %d\n", InitialDataSize, ModeParam);

	// Format
	for(earseAddr=0;earseAddr<0xf000;earseAddr+=0x1000)
	  ISP3_FlashromBlockErase(earseAddr);
	//ISP3_FlashromFormat();
	
	WaitTime_us(2*200);				//4 200 ms
	#ifdef IIC_BURST_MODE
	ISP3_FlashromBurstWrite(0x0000, pInitialData, InitialDataSize, Cl_Null);
	#else
	ISP3_FlashromWrite(0x0000, pInitialData, InitialDataSize, Cl_Null);
	#endif
	WaitTime_us(2*100);				//4 100 ms
	WaitTime_us(2*100);				//4 100 ms

	#ifndef ISP3_MCU_LSC
	// Code copy    
	#ifdef INIT_IIC_BURST_MODE
	System_IICWrite(ISP_IICID_A, 0x01, 0xe0);
	CoreISP3_I2C_Write_Bulk(FlashDEV, 0x29);   //	# flash code mode
	CoreISP3_I2C_Write_Bulk(FlashDEV, 0x19);   //	# flash code mode
	WaitTime_us(2*10);				//4 10 ms
	CoreISP3_I2C_Write_Bulk(FlashDEV, 0x30);   //	# flash code copy address reset
	CoreISP3_I2C_Write_Bulk(FlashDEV, 0x10);   //# flash code copy address release
	CoreISP3_I2C_Write_Bulk(FlashDSizeH, 0xc0);   //	# code ram size: 48KB
	CoreISP3_I2C_Write_Bulk(FlashDSizeL, 0x00);   
	CoreISP3_I2C_Write_Bulk(FlashDown, 0x01);   //	# flash code copy to code ram start
	WaitTime_us(2*50);				//4 50 ms
	CoreISP3_I2C_Write_Bulk(FlashDown, 0x00);   //	# flash code copy to code ram end
	CoreISP3_I2C_Write_Bulk(FlashDEV, 0x00);   //	# code ram mode 
	#else
	CoreISP3_I2C_Write(FlashDEV, 0x29);   //	# flash code mode
	CoreISP3_I2C_Write(FlashDEV, 0x19);   //	# flash code mode
	WaitTime_us(2*10);				//4 10 ms
	CoreISP3_I2C_Write(FlashDEV, 0x30);   //	# flash code copy address reset
	CoreISP3_I2C_Write(FlashDEV, 0x10);   //# flash code copy address release
	CoreISP3_I2C_Write(FlashDSizeH, 0xc0);   //	# code ram size: 48KB
	CoreISP3_I2C_Write(FlashDSizeL, 0x00);   
	CoreISP3_I2C_Write(FlashDown, 0x01);   //	# flash code copy to code ram start
	WaitTime_us(2*50);				//4 50 ms
	CoreISP3_I2C_Write(FlashDown, 0x00);   //	# flash code copy to code ram end
	CoreISP3_I2C_Write(FlashDEV, 0x00);   //	# code ram mode 
	#endif
	#endif
	
#if 1 	//3 Verify MCU Bin Data   // need long time
	printk("MCU Bin Verify START @@@@@@@@@@@@@@@@@@@@@\n");

	#ifdef INIT_IIC_BURST_MODE

	CoreISP3_I2C_Write_Bulk(0xE070,	0x05);		//4 Reset 8051
	WaitTime_us(2*100);				//4 100 ms

	if(ModeParam == enDownloadMode_CodeRAM)
		CoreISP3_I2C_Write_Bulk(0xE0A2,	0xa0);		//4 0x31 : Select Flash memory , 0xA0 : Select SDRAM
	else
		CoreISP3_I2C_Write_Bulk(0xE0A2,	0x31);

	WaitTime_us(2*100);				//4 100 ms

	for( i=0; i<InitialDataSize; i++ ) 
	{
		CoreISP3_I2C_Write_Bulk(0xE0A3,	i>>8);		//4 Set Upper Address
		CoreISP3_I2C_Write_Bulk(0xE0A4, i & 0xFF);	//4 Set Lower Addreaa
		reg_data = CoreISP3_I2C_Read_Bulk(0xE0A5);	//4 Read Data of Address

		if (reg_data != pInitialData[i])
		{
			printk( "MCU Bin Verify ERROR pInitialData[%d] = 0x%x : 0x%x @@@@@@@@@@@@@@@@@@@@@\n", i, reg_data, pInitialData[i]);
			return Cl_False;
		}

	}
	#else
	CoreISP3_I2C_Write(0xE070,	0x05);		//4 Reset 8051
	WaitTime_us(2*100);				//4 100 ms

	if(ModeParam == enDownloadMode_CodeRAM)
		CoreISP3_I2C_Write(0xE0A2,	0xa0);		//4 0x31 : Select Flash memory , 0xA0 : Select SDRAM
	else
		CoreISP3_I2C_Write(0xE0A2,	0x31);

	WaitTime_us(2*100);				//4 100 ms
	printk("!!!!!!!!!!!!!!!!!!!!!!datasize=%d\n",InitialDataSize);
	for( i=0; i<InitialDataSize; i++ ) 
	{       printk("%d\n",i);
		CoreISP3_I2C_Write(0xE0A3,	i>>8);		//4 Set Upper Address
		CoreISP3_I2C_Write(0xE0A4, i & 0xFF);	//4 Set Lower Addreaa
		reg_data = CoreISP3_I2C_Read(0xE0A5);	//4 Read Data of Address

		if (reg_data != pInitialData[i])
		{	while(1)
			printk( "MCU Bin Verify ERROR pInitialData[%d] = 0x%x : 0x%x @@@@@@@@@@@@@@@@@@@@@\n", i, reg_data, pInitialData[i]);
			return Cl_False;
		}

	}
	#endif
	printk( "MCU Bin Verify END @@@@@@@@@@@@@@@@@@@@@\n");	
#else   // must add this	
	CoreISP3_I2C_Write(0xE070,	0x05);		//4 Reset 8051
	WaitTime_us(2*10);				//4 100 ms

	if(ModeParam == enDownloadMode_CodeRAM)
		CoreISP3_I2C_Write(0xE0A2,	0xa0);		//4 0x31 : Select Flash memory , 0xA0 : Select SDRAM
	else
		CoreISP3_I2C_Write(0xE0A2,	0x31);

	WaitTime_us(2*10);				//4 100 ms
#endif
	printk("ok\n");
	return 1;
}


Cl_Bool CoreISP3_PowerOn()
{
	return 0;
}

Cl_Bool CoreISP3_Reset()
{
	return 0;
}

// Following part is  our stardard function api
/*********************************************************************
*Name			:	CoreISP3_Initialize
*Description	:	Main ISP3 Initialize Routine
*Param			:	param	enDownloadMode_StackedMem		Write to Flash
							enDownloadMode_SkipedMCUBin		Read from Flash
							enDownloadMode_CodeRAM			Write to RAM
*return			:	Cl_True : Scuuess, Cl_False : Fail
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
Cl_Bool CoreISP3_Initialize( enIsp3DownloadMode param )
{
	Cl_Bool RetVal	= Cl_True;
	ClUint_32 temp;

	
	printk("=========================== CoreISP3_Initialize : START ===========================\n");
	printk("enIsp3DownloadMode : %d\n", param);

	/* ISP Clock Setting */
	CoreISP3_SetClock();
	
	// LSC Table Download.
	/**/

	
	/* MCU Binary Download */
	if(param != enDownloadMode_SkipedMCUBin)
	{

		RetVal = CoreISP3_MCU_CodeDownload((ClUint_8*)&ISP3BinaryDataMcu, ISP3BinarySize, param);

		if (RetVal == Cl_False)
		{
			printk( "CoreISP3_MCU_CodeDownload Fail Error : %d \n", RetVal);
			return Cl_False;
		}

		
	}

	/* MCU Reset */
	CoreISP3_I2C_Write(MCURST, 0x05);
	WaitTime_us(2*10);    // 100ms  	// Jacky change this time for init time
	CoreISP3_I2C_Write(MCURST, 0x04);
	WaitTime_us(2*500);    // 100ms	// Jacky change this time for init time
	
	/* Send Command(ISP & Sensor Initialize) to ISP */
	printk( "ISP3_CMD_SENSOR_INIT \n");
	CoreISP3_Send_Command(ISP3_CMD_SENSOR_INIT);
		
	WaitTime_us(2*100);    // 100ms
	printk("after  cmd init  0xe062[0x%x], 0xe061[0x%x], 0xe063[0x%x]\n", CoreISP3_I2C_Read(0xe062) , CoreISP3_I2C_Read(0xe061), CoreISP3_I2C_Read(0xe063));
	printk("ISP3_CMD_SENSOR_INIT end\n");

	/*Sensor Interface*/
	CoreISP3_I2C_Write(SMIABypass, 0x00);   //[7:7] LMBYPASS '0' = Async, '1' = Sync
	printk("after  cmd SMIABypass  0xe062[0x%x], 0xe061[0x%x], 0xe063[0x%x]\n", CoreISP3_I2C_Read(0xe062) , CoreISP3_I2C_Read(0xe061), CoreISP3_I2C_Read(0xe063));

	/* OUT FORMAT */
	//CoreISP3_I2C_Write(OutFmt_JPG, 0x82);   //[0:0] OutFmt ' 0' = YCbCr, '1' = JPEG
	CoreISP3_I2C_Write(OutFmt_JPG, 0x0);   //[0:0] OutFmt ' 0' = YCbCr, '1' = JPEG   for yuv
	WaitTime_us(2*10);    // 100ms	// Jacky change this time for init time

	printk( "=========================== CoreISP3_Initialize : END ===========================\n");

	//CoreISP3_Get_Version();	
	//CoreISP3_TestPatten();  // Jacky for test
	
	//CoreISP3_YUV_Swap(enISP_CbYCr);  // Jacky for Meizu test  binary already add
	//CoreISP3_Mirror_Flip(ISP3_ROTATION_NONE);
	//CoreISP3_PCLK_Inv();  // Jacky for Meizu test  binary already add
	
	return 1;
}


void CoreISP3_SetResolution( enIsp3OutResolution OutResolution , CMD_Mode Mode )
{
  printk("SetResolution :%d ,%d \n",OutResolution, Mode);
#if 1
	if(CMD_Preview == Mode) //Fix resolution 
	{
		switch(OutResolution)
		{

		case enISP_RES_1_3MP:   // 1280 x 960
				CoreISP3_I2C_Write(0xE6A8, SXGA);
			break;

		case enISP_RES_XGA:     // 1024 x  768
				CoreISP3_I2C_Write(0xE6A8, XGA);
			break;

		case enISP_RES_SVGA:    // 800 x  600
				CoreISP3_I2C_Write(0xE6A8, SVGA);
			break;
			
		#ifdef MEIZU_FEATURES
		case enISP_RES_QSVGA:    // 400x300
				CoreISP3_I2C_Write(0xE6A8, QSVGA);
			break;
		#endif
		
		case enISP_RES_VGA:     // 640 x  480
				CoreISP3_I2C_Write(0xE6A8, VGA);
			break;

		case enISP_RES_QVGA:    // 320 x  240
				CoreISP3_I2C_Write(0xE6A8, QVGA);      
			break;

		default:
			break;
		}		
		
		CoreISP3_Send_Command(ISP3_CMD_PREVIEW);
		CoreISP3_SetSensorInfo(OutResolution);
		return;
	}
#endif

	switch(OutResolution)
	{
	case enISP_RES_5MP_FULL:    // 2560 x 1920
		{
			CoreISP3_I2C_Write(0xE6A8, QSXGA);
			CoreISP3_I2C_Write(0xE62b, 0x00);//zoom
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);
		}
		break;

	case enISP_RES_QXGA:    // 2048 x 1536
		{		
			CoreISP3_I2C_Write(0xE6A8, QXGA);
			WaitTime_us(2*100);    // 100ms  // Jacky for test
			CoreISP3_Send_Command(0xdf);

			CoreISP3_I2C_Write(0xE6A8, QXGA);
			CoreISP3_I2C_Write(0xE62b, 0x00);//zoom
			WaitTime_us(2*100);    // 100ms  // Jacky for test
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
		}
		break;            

	case enISP_RES_UXGA:     // 1600 x 1200
		{
			CoreISP3_I2C_Write(0xE6A8, UXGA);
			CoreISP3_I2C_Write(0xE62b, 0x00);//zoom
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
		}  
		break;

	case enISP_RES_SXGA:     // 1280 x 1024
		{
			CoreISP3_I2C_Write(0xE6A8, SXGA);
			CoreISP3_I2C_Write(0xE62b, 0x00);//zoom
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
		}  
		break;

	case enISP_RES_1_3MP:   // 1280 x 960
		{
			CoreISP3_I2C_Write(0xE6A8, 4);
			CoreISP3_I2C_Write(0xE62b, 0x00);//zoom
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
		}
		break;

	case enISP_RES_XGA:     // 1024 x  768
		{
			CoreISP3_I2C_Write(0xE6A8, XGA);
			CoreISP3_I2C_Write(0xE62b, 0x00);//zoom
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
		} 
		break;

	case enISP_RES_SVGA:    // 800 x  600
		{
			CoreISP3_I2C_Write(0xE6A8, SVGA);
			CoreISP3_I2C_Write(0xE62b, 0x00);
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
		}
		break;

	case enISP_RES_VGA:     // 640 x  480
		{
			CoreISP3_I2C_Write(0xE6A8, VGA);
			CoreISP3_I2C_Write(0xE62b, 0x00);//zoom
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
		}
		break;

	case enISP_RES_QVGA:    // 320 x  240
		{
			CoreISP3_I2C_Write(0xE6A8, QVGA);      
			CoreISP3_Send_Command(ISP3_CMD_CAPTURE);    
		}
		break;

	default:
		break;
	}  
	/* Store current resolution */
	CoreISP3_SetSensorInfo(OutResolution);

}

/*********************************************************************
*Name			:	CoreISP3_SetAutoWhiteBalance
*Description	:	Set AWB
*Param			:	AWB_Param		ON
									OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetAutoWhiteBalance( enIsp3FunctionsAWB AWB_Param )
{
	printk( "====> %s::%d\n", __func__, AWB_Param);

	if(AWB_Param == enISP_FUNC_AWB_ON)
	{
		/* Send command to CoreISP3_Send_CommandISP MCU */   
		CoreISP3_Send_Command(ISP3_CMD_AWB_ON);
	}
	else //AWB_Param == enISP_FUNC_AWB_OFF
	{
		/* Send command to ISP MCU */    
		CoreISP3_Send_Command(ISP3_CMD_AWB_OFF);
	}
	/* Delay Time : 100ms */
	WaitTime_us(2*100);
}

/*********************************************************************
*Name			:	CoreISP3_SetAutoExposure
*Description	:	Set AE
*Param			:	AE_Param		ON
									OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetAutoExposure( enIsp3FunctionsAE AE_Param )
{
	printk("====> %s::%d\n", __func__, AE_Param);

	if(AE_Param == enISP_FUNC_AE_ON)
	{
		/* Send command to ISP MCU */
		CoreISP3_Send_Command(ISP3_CMD_AE_ON);
	}
	else //AE_Param == enISP_FUNC_AE_OFF
	{
		/* Send command to ISP MCU */
		CoreISP3_Send_Command(ISP3_CMD_AE_OFF);
	}
	/* Delay Time : 100ms */
	WaitTime_us(2*100);
}
/*********************************************************************
*Name			:	CoreISP3_SetAutoFocus
*Description	:	Set AF
*Param			:	AF_Param		ON
									OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetAutoFocus( enIsp3FunctionsAF AF_Param )
{
	ClUint_16 af_value;
	
#ifdef MEIZU_FEATURES	
	printk("0xE072: 0x%x \n", CoreISP3_I2C_Read(0xE072));
	//CoreISP3_I2C_Write(0xE072, 0xff);
	printk( "...0xE072: 0x%x \n", CoreISP3_I2C_Read(0xE072));
#endif

	printk( "====> %s::%d\n", __func__, AF_Param);
	//CoreISP3_I2C_Write(0xE072, 0xff);

	if(AF_Param == enISP_FUNC_AF_ON)
	{
		/* Send command to ISP MCU */
		//CoreISP3_Send_Command(ISP3_CMD_FULLAF);
		CoreISP3_Send_Command(ISP3_CMD_AF_ON);
	}
	else //AF_Param == enISP_FUNC_AF_OFF
	{
		/* Send command to ISP MCU */
		CoreISP3_Send_Command(ISP3_CMD_AF_OFF);
	}
	/* Delay Time : 100ms */
	//WaitTime_us(2*100);

/*
1. AF Complete Register [0xE661]
 -> Yulong updated AF value before read AF done.
 calculate--> Only use this register for AF Complete. (Yulong no need to change SW)
*/	
	//af_value = CoreISP3_I2C_Read(0xE661);
	//printk( " af value 0xE661 = 0x%x\n", af_value);
	
	//WaitTime_us(2*500);  // 1S wait
	//WaitTime_us(2*500);
	//CoreISP3_GetAutoFocusState();
	
}

/*********************************************************************
*Name			:	CoreISP3_SetAutoFocus_FullScan
*Description	:		Set AF
*Param			:	void
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetAutoFocus_FullScan(void)
{
	printk("====> %s::\n", __func__);

	/* Send command to ISP MCU */
	CoreISP3_Send_Command(ISP3_CMD_AF_FULL_SCAN);
	
	//WaitTime_us(2*100);
}

/*********************************************************************
*Name			:	CoreISP3_SetAutoFocus_Micro
*Description	:		Set AF
*Param			:	void
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetAutoFocus_Micro(void)//微距离
{
	printk( "====> %s::\n", __func__);

	/* Send command to ISP MCU */
	CoreISP3_Send_Command(ISP3_CMD_MICROAF);
	
	//WaitTime_us(2*100);
}

/*
2. JPEG EXIF Information.
 -> ISO Value :
    0xE649[7:0] = ISO Gain 
    calculate =>  ( ISO Gain / 8 ) x 100 = ISO Value

*/
/*********************************************************************
*Name			:	CoreISP3_SetAutoFocus_FullScan
*Description	:		Set AF
*Param			:	void
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
ClUint_32 CoreISP3_GetISOGain(void)
{
	ClUint_32 gain;
	ClUint_16 val;

	val = CoreISP3_I2C_Read(0xE649);
	
	gain = val/8 *100;
	printk( "\n====> vol: [%x]:: Gain: [%x] \n", val, gain);
		
	WaitTime_us(2*100);

	return gain;
}

/*********************************************************************
*Name			:	CoreISP3_SetAutoFocus_FullScan
*Description	:		Set AF
*Param			:	void
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
 -> Shutter Speed : 
      0xE64A [7:0] = Shutter Speed high byte, 0xE64b [7:0] = Shutter Speed  low byte, 0xE663[7:0] = Hsync Time.
    calculate= >  Shutter Speed ( high byte, low byte) x Hsync Time = Shutter Speed [ uS ]
 
**********************************************************************/
ClUint_32 CoreISP3_GetShutterSpeed(void)
{
	ClUint_32 speed;
	ClUint_16 high;
	ClUint_16 low;
	ClUint_16 Hsync_time;
	
	printk( "====> %s::\n", __func__);

	high = CoreISP3_I2C_Read(0xE64A);
	low = CoreISP3_I2C_Read(0xE64B);
	Hsync_time = CoreISP3_I2C_Read(0xE663);
	
	printk( "====> %x  %x  %x::\n", high, low, Hsync_time);
	
	speed = (((high&0xff)<<8)|(low&0xff))*Hsync_time;
	
	WaitTime_us(2*100);

	return speed;
}

ClUint_32 CoreISP3_GetAutoFocusState(void)
{
	ClUint_32 af_state;
	// When af this value is 0x23, when af finish this value is not 0x23    0x24 finished, 0x23 not focused
	af_state = CoreISP3_I2C_Read(0xE661);
	
	printk( "====> af status  %d::\n", af_state);
	return af_state;
}

/*********************************************************************
*Name			:	CoreISP3_SetANR
*Description	:	Set ANR
*Param			:	ANR_Param		enISP_FUNC_ANR_LEVEL_0
									enISP_FUNC_ANR_LEVEL_1
									enISP_FUNC_ANR_LEVEL_2
									enISP_FUNC_ANR_LEVEL_3
									enISP_FUNC_ANR_LEVEL_4
*return			:	none
*Author			:	
*Remark			:	enISP_FUNC_ANR_LEVEL_0 is ANR OFF.
*Log			:	
**********************************************************************/
void CoreISP3_SetANR( enISP3FunctionsANRLevel ANR_Param )
{
	printk( "====> %s::%d\n", __func__, ANR_Param);

	/* Set Adative Noise Reduction Level */
	switch(ANR_Param)
	{
		case enISP_FUNC_ANR_LEVEL_0:    /*Auto*/
			CoreISP3_I2C_Write(0xe671, 0x4);
			break;
		case enISP_FUNC_ANR_LEVEL_1:
		case enISP_FUNC_ANR_LEVEL_2:
		case enISP_FUNC_ANR_LEVEL_3:
		case enISP_FUNC_ANR_LEVEL_4:
			CoreISP3_I2C_Write(0xe671, ANR_Param);
			break;
	}
	/* Send command to ISP MCU */
	CoreISP3_Send_Command(ISP3_CMD_ANR_ON);

	/* Delay Time : 100ms */
	WaitTime_us(2*100);
}
/*********************************************************************
*Name			:	CoreISP3_SetWDR//宽通带
*Description	:	Set WDR
*Param			:	stIspWdrCtrl		bWDREnable					WDR ON/OFF
										enISP_FUNC_WDR_LEVEL_0
									    enISP_FUNC_WDR_LEVEL_1
									    enISP_FUNC_WDR_LEVEL_2
									    enISP_FUNC_WDR_LEVEL_3
									    enISP_FUNC_WDR_LEVEL_4
									    enISP_FUNC_WDR_LEVEL_5
									    enISP_FUNC_WDR_LEVEL_6
									    enISP_FUNC_WDR_LEVEL_7
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetWDR( _tISP_WDR_CTRL *stIspWdrCtrl )
{
	printk( "====> %s::%d,%d\n", __func__, stIspWdrCtrl->bWDREnable, stIspWdrCtrl->WDRlevel);

	if(stIspWdrCtrl->bWDREnable)
	{
		/* Set WDR Level */
		CoreISP3_I2C_Partial_Write(0xe006, 7, 5, stIspWdrCtrl->WDRlevel);

		/* WDR Enable */
		CoreISP3_I2C_Partial_Write(0xe006, 4, 4, stIspWdrCtrl->bWDREnable);
	}
	else
	{
		/* WDR Disable */
		CoreISP3_I2C_Partial_Write(0xe006, 4, 4, stIspWdrCtrl->bWDREnable);
	}
	/* Delay Time : 100ms */
	WaitTime_us(2*100);
}

/*********************************************************************
*Name			:	CoreISP3_StillStabilizerLevelSetup
*Description	:	Set Stabilizer's Level
*Param			:	enISP_FUNC_SS_LEVEL_1
				    enISP_FUNC_SS_LEVEL_1_2
				    enISP_FUNC_SS_LEVEL_1_3
				    enISP_FUNC_SS_LEVEL_1_4
				    enISP_FUNC_SS_LEVEL_1_5
				    enISP_FUNC_SS_LEVEL_1_6
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_StillStabilizerLevelSetup( ClUint_16 LevelParam )
{
    printk( "====> %s::%d\n", __func__, LevelParam);
     
    switch(LevelParam)
    {
        case enISP_FUNC_SS_LEVEL_NONE:
        case enISP_FUNC_SS_LEVEL_1:
            break;            
        case enISP_FUNC_SS_LEVEL_1_2:
        case enISP_FUNC_SS_LEVEL_1_3:
        case enISP_FUNC_SS_LEVEL_1_4:
        case enISP_FUNC_SS_LEVEL_1_5:
        case enISP_FUNC_SS_LEVEL_1_6:
            CoreISP3_I2C_Write(0xe670, LevelParam);
            CoreISP3_I2C_Write(0xe671, 0x00);
            break;
    }
}

/*********************************************************************
*Name			:	CoreISP3_SetStillStabilizer
*Description	:	Set Stabilizer
*Param			:	stIspStillStabilizerCtrlParam		bStillStabilizerEnable		Stabilizer ON/OFF
													    enISP_FUNC_SS_LEVEL_1
													    enISP_FUNC_SS_LEVEL_1_2
													    enISP_FUNC_SS_LEVEL_1_3
													    enISP_FUNC_SS_LEVEL_1_4
													    enISP_FUNC_SS_LEVEL_1_5
													    enISP_FUNC_SS_LEVEL_1_6
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetStillStabilizer( _tIspStillStabilizerCtrl *stIspStillStabilizerCtrlParam )
{
    printk("====> %s::%d,%d\n", __func__, stIspStillStabilizerCtrlParam->bStillStabilizerEnable, stIspStillStabilizerCtrlParam->StillStabilzerLevel);

    if(stIspStillStabilizerCtrlParam->bStillStabilizerEnable)
    {
        /* Still stabilizer enable */
        CoreISP3_I2C_Write(0xe004, 0x2b);

        /* Set the SS level */
        CoreISP3_StillStabilizerLevelSetup(stIspStillStabilizerCtrlParam->StillStabilzerLevel);

        /* Send command to ISP MCU : Still Stabilizer Enable */
        CoreISP3_Send_Command(ISP3_CMD_STILL_ON);
    }
    else
    {
        /* Still stabilizer disable */
        CoreISP3_I2C_Write(0xe004, 0x22);

        /* Send command to ISP MCU : Still Stabilizer Disable */
        CoreISP3_Send_Command(ISP3_CMD_STILL_OFF);
    }
}
/*********************************************************************
*Name			:	CoreISP3_SetStillStabilizer
*Description	:	Set Stabilizer
*Param			:	enISP_FUNC_IMAGE_NORMAL
				    enISP_FUNC_IMAGE_EMBOSS
				    enISP_FUNC_IMAGE_SKETCH1
				    enISP_FUNC_IMAGE_SKETCH2
				    enISP_FUNC_IMAGE_BLACK_WHITE
				    enISP_FUNC_IMAGE_NORMAL_MOVIE
				    enISP_FUNC_IMAGE_OLD_MOVIE
				    enISP_FUNC_IMAGE_GRAY
				    enISP_FUNC_IMAGE_ACCENT
				    enISP_FUNC_IMAGE_SWAPING
				    enISP_FUNC_IMAGE_ACCENT_SWAPING
				    enISP_FUNC_IMAGE_WARM
				    enISP_FUNC_IMAGE_COOL
				    enISP_FUNC_IMAGE_FOG
				    enISP_FUNC_IMAGE_OPPOSITE_NEGATIVE
				    enISP_FUNC_IMAGE_OPPOSITE_AVERAGE
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetImageEffect( enISPFunctionsImageEffect IspImageEffectParam )
{
    printk( "====> %s::%d\n", __func__, IspImageEffectParam);

    switch(IspImageEffectParam)
    {
        case enISP_FUNC_IMAGE_NORMAL:
            CoreISP3_I2C_Write(ImgEffectA, 0x02);
            CoreISP3_I2C_Write(ImgEffectB, 0x00);
            CoreISP3_I2C_Write(ImgEffectC, 0x00);            
            break;
        case enISP_FUNC_IMAGE_EMBOSS:
            CoreISP3_I2C_Write(ImgEffectC, 0x01);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On
            break;            
        case enISP_FUNC_IMAGE_SKETCH1:
            CoreISP3_I2C_Write(ImgEffectC, 0x02);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect Ons            
            break;            
        case enISP_FUNC_IMAGE_SKETCH2:
            CoreISP3_I2C_Write(ImgEffectC, 0x03);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;            
        case enISP_FUNC_IMAGE_BLACK_WHITE:
            CoreISP3_I2C_Write(ImgEffectC, 0x04);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;            
        case enISP_FUNC_IMAGE_NORMAL_MOVIE:
            CoreISP3_I2C_Write(ImgEffectC, 0x05);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On
            break;            
        case enISP_FUNC_IMAGE_OLD_MOVIE:
            CoreISP3_I2C_Write(ImgEffectC, 0x06);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;
        case enISP_FUNC_IMAGE_GRAY:
            CoreISP3_I2C_Write(ImgEffectC, 0x07);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;
        case enISP_FUNC_IMAGE_ACCENT:
            CoreISP3_I2C_Write(Acc_Cb_Min, 0x00);
            CoreISP3_I2C_Write(Acc_Cb_Max, 0xff);
            CoreISP3_I2C_Write(Acc_Cr_Min, 0x00);
            CoreISP3_I2C_Write(Acc_Cr_Max, 0x60);
            CoreISP3_I2C_Write(ImgEffectC, 0x08);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;            
        case enISP_FUNC_IMAGE_SWAPING:
            CoreISP3_I2C_Write(Swp_Cb_Min1, 0x80);
            CoreISP3_I2C_Write(Swp_Cb_Max1, 0xff);
            CoreISP3_I2C_Write(Swp_Cr_Min1, 0x20);
            CoreISP3_I2C_Write(Swp_Cr_Max1, 0x60);
            CoreISP3_I2C_Write(Swp_Cb1, 0x00);
            CoreISP3_I2C_Write(Swp_Cr1, 0xff);
            CoreISP3_I2C_Write(Swp_Cb_Min2, 0x20);
            CoreISP3_I2C_Write(Swp_Cb_Max2, 0x60);
            CoreISP3_I2C_Write(Swp_Cr_Min2, 0x80);
            CoreISP3_I2C_Write(Swp_Cr_Max2, 0xff);
            CoreISP3_I2C_Write(Swp_Cb2, 0xff);
            CoreISP3_I2C_Write(Swp_Cr2, 0x00);            
            CoreISP3_I2C_Write(ImgEffectC, 0x09);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On
            break;            
        case enISP_FUNC_IMAGE_ACCENT_SWAPING:
            CoreISP3_I2C_Write(ASwp_Cb_Min1, 0x80);
            CoreISP3_I2C_Write(ASwp_Cb_Max1, 0xff);
            CoreISP3_I2C_Write(ASwp_Cr_Min1, 0x20);
            CoreISP3_I2C_Write(ASwp_Cr_Max1, 0x60);
            CoreISP3_I2C_Write(ASwp_Cb1, 0x00);
            CoreISP3_I2C_Write(ASwp_Cr1, 0xff);
            CoreISP3_I2C_Write(ASwp_Cb_Min2, 0x20);
            CoreISP3_I2C_Write(ASwp_Cb_Max2, 0x60);
            CoreISP3_I2C_Write(ASwp_Cr_Min2, 0x80);
            CoreISP3_I2C_Write(ASwp_Cr_Max2, 0xff);
            CoreISP3_I2C_Write(ASwp_Cb2, 0xff);
            CoreISP3_I2C_Write(ASwp_Cr2, 0x00);                        
            CoreISP3_I2C_Write(ImgEffectC, 0x0a);
            CoreISP3_I2C_Write(ImgEffectA, 0x04); //YCbCr Effect On            
            break;            
        case enISP_FUNC_IMAGE_WARM:
            CoreISP3_I2C_Write(ImgEffectB, 0x01);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On            
            break;            
        case enISP_FUNC_IMAGE_COOL:
            CoreISP3_I2C_Write(ImgEffectB, 0x02);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On                        
            break;            
        case enISP_FUNC_IMAGE_FOG:
		CoreISP3_I2C_Write(ImgEffectB, 0x03);
		CoreISP3_I2C_Write(ImgEffectD, 0x80);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On                        
            break;            
        case enISP_FUNC_IMAGE_OPPOSITE_NEGATIVE:  //反色
            CoreISP3_I2C_Write(ImgEffectB, 0x04);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On                        
            break;            
        case enISP_FUNC_IMAGE_OPPOSITE_AVERAGE:
            CoreISP3_I2C_Write(ImgEffectB, 0x05);
            CoreISP3_I2C_Write(ImgEffectA, 0x00); //RGB Effect On                        
            break;
        default:
        case 'x': case 'X':
            break;
    }
}

/*********************************************************************
*Name			:	CoreISP3_SetFaceTracking
*Description	:	Set Face Tracking
*Param			:	FaceTrackingParam	ON
										OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceTracking( Cl_Bool FaceTrackingParam )
{
    printk( "====> %s::%d\n", __func__, FaceTrackingParam);

    if(FaceTrackingParam)
    {
        /* Send command to ISP MCU : Face Tracking Enable  */
        CoreISP3_Send_Command(ISP3_CMD_FACE_TRACKING_START);
    }
    else
    {
        /* Send command to ISP MCU : Face Tracking Disable */
        CoreISP3_Send_Command(ISP3_CMD_FACE_TRACKING_STOP);
    }
    /* Delay Time : 100ms */
    WaitTime_us(2*100);
}
/*********************************************************************
*Name			:	CoreISP3_SetFaceAE
*Description	:	Set AE with Face Tracking
*Param			:	FaceAEParam			ON
										OFF
					FaceAEGlobalParam	ON
										OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceAE( Cl_Bool FaceAEParam)
{
	printk( "====> %s::%d, %d\n", __func__, FaceAEParam);
#if 0
    if(FaceAEParam)
    {
        /* Send command to ISP MCU : Face AE Enable  */
        CoreISP3_Send_Command(ISP3_CMD_FACE_AE_START);
    }
    else 
    {
        /* Send command to ISP MCU : Face AE Disable */
        CoreISP3_Send_Command(ISP3_CMD_FACE_AE_STOP);
    }
#else
	switch(FaceAEParam)
	{
		case enISP_FUNC_AEwithFACE_1:
			CoreISP3_Send_Command(ISP3_CMD_FACE_AE_STOP);
			break;
		case enISP_FUNC_AEwithFACE_2:
			CoreISP3_Send_Command(ISP3_CMD_FACE_AE_START);
			CoreISP3_Send_Command(ISP3_CMD_AE_GLOBAL);
			break;
		case enISP_FUNC_AEwithFACE_3:
			CoreISP3_Send_Command(ISP3_CMD_FACE_AE_START);
			CoreISP3_Send_Command(ISP3_CMD_AE_MULTI);
			break;
	}
	/* Delay Time : 100ms */
	WaitTime_us(2*100);
#endif
}
/*********************************************************************
*Name			:	CoreISP3_SetFaceAF
*Description	:	Set AF with Face Tracking
*Param			:	FaceAFParam			ON
										OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceAF( Cl_Bool FaceAFParam )
{
    printk( "====> %s::%d\n", __func__, FaceAFParam);

    if(FaceAFParam)
    {
        /* Send command to ISP MCU : Face AF Enable  */
        CoreISP3_Send_Command(ISP3_CMD_FACE_AF_START);

        /* Delay Time : 100ms */
        WaitTime_us(2*100);

        /* Send command to ISP MCU : Face Full AF Enable  */
        CoreISP3_Send_Command(ISP3_CMD_FULLAF);
    }
    else 
    {
        /* Send command to ISP MCU : Face AF Disable */
        CoreISP3_Send_Command(ISP3_CMD_FACE_AF_STOP);
    }
    /* Delay Time : 100ms */
    WaitTime_us(2*100);
}
/*********************************************************************
*Name			:	CoreISP3_SetFaceAWB
*Description	:	Set AWB with Face Tracking
*Param			:	enISP_FUNC_AWBwithFD_Level_1
				    enISP_FUNC_AWBwithFD_Level_2
				    enISP_FUNC_AWBwithFD_Level_3
				    enISP_FUNC_AWBwithFD_Level_4
				    enISP_FUNC_AWBwithFD_Level_5    
				    enISP_FUNC_AWBwithFD_Level_6
				    enISP_FUNC_AWBwithFD_Level_7
				    enISP_FUNC_AWBwithFD_Level_8    
				    enISP_FUNC_AWBwithFD_Level_9    
				    enISP_FUNC_AWBwithFD_Level_10    
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceAWB( Cl_Bool FaceAWBParam )
{
#if 0
    if(FaceAWBParam)
    {
    }
    else 
    {
    }
#else
	ClUint_16 retval = 0;
	printk("====> %s::%d\n", __func__, FaceAWBParam);
	switch(FaceAWBParam)
	{
		case enISP_FUNC_AWBwithFD_Level_1:
			retval = CoreISP3_I2C_Read(EnAWBFaceH);
			CoreISP3_I2C_Write(EnAWBFaceH, (retval | 0x02));
			break;
		case enISP_FUNC_AWBwithFD_Level_2:
			retval = CoreISP3_I2C_Read(EnAWBFaceH);
			CoreISP3_I2C_Write(EnAWBFaceH, (retval | 0x01));
			break;
		case enISP_FUNC_AWBwithFD_Level_3:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x80));
			break;
		case enISP_FUNC_AWBwithFD_Level_4:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x40));
			break;
		case enISP_FUNC_AWBwithFD_Level_5:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x20));
			break;
		case enISP_FUNC_AWBwithFD_Level_6:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x10));
			break;
		case enISP_FUNC_AWBwithFD_Level_7:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x08));
			break;
		case enISP_FUNC_AWBwithFD_Level_8:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x04));
			break;
		case enISP_FUNC_AWBwithFD_Level_9:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x02));
			break;
		case enISP_FUNC_AWBwithFD_Level_10:
			retval = CoreISP3_I2C_Read(EnAWBFaceL);
			CoreISP3_I2C_Write(EnAWBFaceL, (retval | 0x01));
			break;
		default:
			break;
	}
#endif
}
/*********************************************************************
*Name			:	CoreISP3_SetFaceRotation
*Description	:	Set Rotation of Face Tracking
*Param			:	FaceRotation		ON
										OFF
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceRotation( Cl_Bool FaceRotation )
{
    printk( "====> %s::%d\n", __func__, FaceRotation);

    if(FaceRotation)
    {
        /* Send command to ISP MCU : Face Rotation Enable  */
        CoreISP3_Send_Command(ISP3_CMD_FACE_ROTATE_START);
    }
    else 
    {
        /* Send command to ISP MCU : Face Rotation Disable */
        CoreISP3_Send_Command(ISP3_CMD_FACE_ROTATE_STOP);
    }
    /* Delay Time : 100ms */
    WaitTime_us(2*100);
}
/*********************************************************************
*Name			:	CoreISP3_SetFaceROIThick
*Description	:	Set ROI of Face Tracking
*Param			:	FaceRoiThickParam		enISP_FUNC_FD_ROI_THICK_0
										    enISP_FUNC_FD_ROI_THICK_1
										    enISP_FUNC_FD_ROI_THICK_2
										    enISP_FUNC_FD_ROI_THICK_3
										    enISP_FUNC_FD_ROI_THICK_4
										    enISP_FUNC_FD_ROI_THICK_5
										    enISP_FUNC_FD_ROI_THICK_6
										    enISP_FUNC_FD_ROI_THICK_7
										    enISP_FUNC_FD_ROI_THICK_8
										    enISP_FUNC_FD_ROI_THICK_9
										    enISP_FUNC_FD_ROI_THICK_10
										    enISP_FUNC_FD_ROI_THICK_11
										    enISP_FUNC_FD_ROI_THICK_12
										    enISP_FUNC_FD_ROI_THICK_13
										    enISP_FUNC_FD_ROI_THICK_14
										    enISP_FUNC_FD_ROI_THICK_15
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceROIThick( enIsp3FaceROIThick FaceRoiThickParam )
{
    printk("====> %s::%d\n", __func__, FaceRoiThickParam);

    CoreISP3_I2C_Write(0xf20c, FaceRoiThickParam);
    printk(">>ISP2_REG>>0xf20c==0x%x\n",CoreISP3_I2C_Read(0xf20c));
}
/*********************************************************************
*Name			:	CoreISP3_SetFaceMaxDetectionCount
*Description	:	Set Max Detecting count of Face Tracking
*Param			:	FaceMaxDetectionParam		enISP_FUNC_FD_MAXCOUNT_1
											    enISP_FUNC_FD_MAXCOUNT_2
											    enISP_FUNC_FD_MAXCOUNT_3
											    enISP_FUNC_FD_MAXCOUNT_4
											    enISP_FUNC_FD_MAXCOUNT_5    
											    enISP_FUNC_FD_MAXCOUNT_6
											    enISP_FUNC_FD_MAXCOUNT_7
											    enISP_FUNC_FD_MAXCOUNT_8    
											    enISP_FUNC_FD_MAXCOUNT_9    
											    enISP_FUNC_FD_MAXCOUNT_10    
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceMaxDetectionCount( enIsp3FaceMaxDetectionCount FaceMaxDetectionParam )
{
    printk( "====> %s::%d\n", __func__, FaceMaxDetectionParam);

    switch(FaceMaxDetectionParam)
    {
        case enISP_FUNC_FD_MAXCOUNT_1:
            CoreISP3_I2C_Write(0xf20a, 0x02);
            CoreISP3_I2C_Write(0xf20b, 0x00);
            break;
        case enISP_FUNC_FD_MAXCOUNT_2:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0x00);
            break;
        case enISP_FUNC_FD_MAXCOUNT_3:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0x80);
            break;
        case enISP_FUNC_FD_MAXCOUNT_4:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xc0);
            break;
        case enISP_FUNC_FD_MAXCOUNT_5:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xe0);
            break;
        case enISP_FUNC_FD_MAXCOUNT_6:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xf0);
            break;
        case enISP_FUNC_FD_MAXCOUNT_7:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xf8);
            break;
        case enISP_FUNC_FD_MAXCOUNT_8:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xfc);
            break;
        case enISP_FUNC_FD_MAXCOUNT_9:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xfe);
            break;
        case enISP_FUNC_FD_MAXCOUNT_10:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xff);
            break;            
        default:
            CoreISP3_I2C_Write(0xf20a, 0x03);
            CoreISP3_I2C_Write(0xf20b, 0xc0);
            break;
    }
    
}

stFaceInfo CoreISP3_GetFacePosition(unsigned char index )
{

	stFaceInfo faceInfo;
	ClUint_16 high;
	ClUint_16 low;
	ClUint_16 regAdd = 0xf3f1 + (index*8);  //start from 0xf3f1, every 8 register
	
	printk( "\n\n====> %s:%d:\n", __func__,CoreISP3_I2C_Read(0xf212));

	high = CoreISP3_I2C_Read(regAdd++);
	low = CoreISP3_I2C_Read(regAdd++);
	faceInfo.sx = (((high&0xff)<<8)|(low&0xff));
       faceInfo.sx/=4;
	printk( "Start X-Coordinate of  face::[%d]::\n", faceInfo.sx);

	high = CoreISP3_I2C_Read(regAdd++);
	low = CoreISP3_I2C_Read(regAdd++);
	faceInfo.sy = (((high&0xff)<<8)|(low&0xff));
       faceInfo.sy/=4;
	printk( "Start Y-Coordinate of  face::[%d]::\n", faceInfo.sy);	

	high = CoreISP3_I2C_Read(regAdd++);
	low = CoreISP3_I2C_Read(regAdd++);
	faceInfo.ex = (((high&0xff)<<8)|(low&0xff));
       faceInfo.ex/=4;
	printk( "END X-Coordinate of  face::[%d]::\n", faceInfo.ex);

	high = CoreISP3_I2C_Read(regAdd++);
	low = CoreISP3_I2C_Read(regAdd++);
	faceInfo.ey = (((high&0xff)<<8)|(low&0xf8));	
       faceInfo.ey/=4;
	printk( "END Y-Coordinate of  face::[%d]::\n", faceInfo.ey);    
	
	return faceInfo;


}


void CoreISP3_Select_ROI(ClUint_8 area, ClUint_8 count)//ROI 第几个人脸用于对焦
{
    ClUint_8 data = (count <<4) | (area & 0x0F);
    CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
    CoreISP3_I2C_Write(0xE640, 0x00);
    CoreISP3_I2C_Write(0xE640, data);
    CoreISP3_Send_Command(0xa9);
    printk( "FaceUserModeParam count:%x  area:%x ==>:%x\n", count,  area,data );
    printk( "Check:%x\n",CoreISP3_I2C_Read(0xE640) );
}

/*********************************************************************
*Name			:	CoreISP3_SetFaceUserMode
*Description	:	Set Face Area to select Face by user
*Param			:	enIspFaceUserMode		enISP_FUNC_FDUser_Level_0 : Disable User Mode
											enISP_FUNC_FDUser_Level_1
											enISP_FUNC_FDUser_Level_2
											enISP_FUNC_FDUser_Level_3
											enISP_FUNC_FDUser_Level_4
											enISP_FUNC_FDUser_Level_5    
											enISP_FUNC_FDUser_Level_6
											enISP_FUNC_FDUser_Level_7
											enISP_FUNC_FDUser_Level_8    
											enISP_FUNC_FDUser_Level_9    
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SetFaceUserMode( enIspFaceUserMode FaceUserModeParam )
{
	ClUint_8 area = 0;
	ClUint_8 count1 = 1, count2 = 1, count3 = 1, count4 = 1, count5 = 1, count6 = 1, count7 = 1, count8 = 1, count9 = 1; 

	#ifdef ISP3_EVB_ENABLE
    	printk( "====> %s::%d\n", __func__, FaceUserModeParam);
	#endif
    #if 1
    CoreISP3_Select_ROI((FaceUserModeParam&0xF),(FaceUserModeParam>>4));
    return;	
    #endif	
	
    switch(FaceUserModeParam)
    {
        case enISP_FUNC_FDUser_Level_1:
			area = enISP_FUNC_FDUser_Level_1;
			
			count1++;
			if(count1 > 5)	count1 = 1;

			#ifdef ISP3_EVB_ENABLE
			printk( "FaceUserModeParam 0x%x\n", (count1 & 0xF0) | (area & 0x0F));
			#endif
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
			CoreISP3_I2C_Write(0xE640, (count9 & 0xF0) | (area & 0x0F));
            break;
			
        case enISP_FUNC_FDUser_Level_2:
			area = enISP_FUNC_FDUser_Level_2;
			count2++;
			if(count2 > 5)	count2 = 1;
			
			#ifdef ISP3_EVB_ENABLE
			printk( "FaceUserModeParam 0x%x\n", (count2 & 0xF0) | (area & 0x0F));
			#endif
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count2 & 0xF0) | (area & 0x0F));
            break;
			
        case enISP_FUNC_FDUser_Level_3:
			area = enISP_FUNC_FDUser_Level_3;
			count3++;
			if(count3 > 5)	count3 = 1;

			#ifdef ISP3_EVB_ENABLE
			printk( "FaceUserModeParam 0x%x\n", (count3 & 0xF0) | (area & 0x0F));
			#endif
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count3 & 0xF0) | (area & 0x0F));
            break;
			
        case enISP_FUNC_FDUser_Level_4:
			area = enISP_FUNC_FDUser_Level_4;
			count4++;
			if(count4 > 5)	count4 = 1;

			#ifdef ISP3_EVB_ENABLE
			printk( "FaceUserModeParam 0x%x\n", (count4 & 0xF0) | (area & 0x0F));
			#endif
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count4 & 0xF0) | (area & 0x0F));
            break;
			
        case enISP_FUNC_FDUser_Level_5:
			area = enISP_FUNC_FDUser_Level_5;
			count5++;
			if(count5> 5)	count5 = 1;
			
			count5<<=4; // Jacky for test
			
			#ifdef ISP3_EVB_ENABLE
			printk( "FaceUserModeParam 0x%x\n", (count5 & 0xF0) | (area & 0x0F));
			#endif
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count5 & 0xF0) | (area & 0x0F));
            break;
			
        case enISP_FUNC_FDUser_Level_6:
			area = enISP_FUNC_FDUser_Level_6;
			count6++;
			if(count6 > 5)	count6 = 1;

			#ifdef ISP3_EVB_ENABLE
			printk( "FaceUserModeParam 0x%x\n", (count6 & 0xF0) | (area & 0x0F));
			#endif
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count6 & 0xF0) | (area & 0x0F));
            break;
        case enISP_FUNC_FDUser_Level_7:
			area = enISP_FUNC_FDUser_Level_7;
			count7++;
			if(count7 > 5)	count7 = 1;

			#ifdef ISP3_EVB_ENABLE
			printk( "FaceUserModeParam 0x%x\n", (count7 & 0xF0) | (area & 0x0F));
			#endif
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count7 & 0xF0) | (area & 0x0F));
            break;
        case enISP_FUNC_FDUser_Level_8:
			area = enISP_FUNC_FDUser_Level_8;
			count8++;
			if(count8 > 5)	count8 = 1;

			#ifdef ISP3_EVB_ENABLE
			printk( "FaceUserModeParam 0x%x\n", (count8 & 0xF0) | (area & 0x0F));
			#endif
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count8 & 0xF0) | (area & 0x0F));
            break;
        case enISP_FUNC_FDUser_Level_9:
			area = enISP_FUNC_FDUser_Level_9;
			count9++;
			if(count9 > 5)	count9 = 1;

			#ifdef ISP3_EVB_ENABLE
			printk( "FaceUserModeParam 0x%x\n", (count9 & 0xF0) | (area & 0x0F));
			#endif
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_START);
			CoreISP3_I2C_Write(0xE640, 0x00);
            		CoreISP3_I2C_Write(0xE640, (count9 & 0xF0) | (area & 0x0F));
            break;
        case enISP_FUNC_FDUser_Level_0:
            		CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_STOP);
			CoreISP3_I2C_Write(0xE640, 0x00);
            break;         
			
        default:
			CoreISP3_Send_Command(ISP3_CMD_FACE_USERMODE_STOP);
			CoreISP3_I2C_Write(0xE640, 0x00);
            break;
    }
    
}

void CoreISP3_FaceSelect(int x, int y)
{
#if 0
	ISP3_RegWrite(0xE628, x>>8);
	ISP3_RegWrite(0xE629, x&0xFF);
	ISP3_RegWrite(0xE62A, y>>8);
	ISP3_RegWrite(0xE62B, y&0xFF);
	ISP3_WriteCmd(FACE_SELECT);   // 0xea
#else

#define ButtonReg		0xE640
#define	ClickPosX		0xE626
#define ClickPosY		0xE627
//#define	FACE_USERMODE_DYNAMIC_AREA			0xa8

	CoreISP3_I2C_Write(ButtonReg, x>>8); // x pos high bit
	CoreISP3_I2C_Write(ClickPosX, x&0xFF);  // y pos low  bit 
	CoreISP3_I2C_Write(ClickPosY, y);
	CoreISP3_Send_Command(0xa8);
#endif
}

void CoreISP3_SetFocusPos(signed short focusPos)//设定步进马达位置 
{
	CoreISP3_I2C_Write(0xE628, (focusPos>>8)&0xFF);
	CoreISP3_I2C_Write(0xE629, focusPos&0xFF);

	CoreISP3_I2C_Write(0xE62A, 0);
	CoreISP3_Send_Command(0xb7);
}

signed short CoreISP3_GetFocusPos(void) 
{
	signed short focusPos;
	
	CoreISP3_I2C_Write(0xE62A, 0);
	CoreISP3_Send_Command(0x4f);

	focusPos = (CoreISP3_I2C_Read(0xE628)<<8 | CoreISP3_I2C_Read(0xE629));
	return focusPos;
}

void CoreISP3_UserArea_AFOn(int centerX, int centerY, int afWindowWidth, int afWindowHeight)
{
//	int afWindowWidth = g_sensorOutputWidth/4;
//	int afWindowHeight = g_sensorOutputHeight/4;
	int sx, sy;
	int ex, ey;
	
	if(centerX - afWindowWidth/2 < 0)
		centerX = afWindowWidth/2;
	else if(centerX + afWindowWidth/2 > 640)
		centerX = 480 - afWindowWidth/2;

	if(centerY - afWindowHeight/2 < 0)
		centerY = afWindowHeight/2;
	else if(centerY + afWindowHeight/2 > 640)
		centerY = 480 - afWindowHeight/2;

	sx = centerX - afWindowWidth/2;
	sy = centerY - afWindowHeight/2;
	ex = centerX + afWindowWidth/2;
	ey = centerY + afWindowHeight/2;

	CoreISP3_I2C_Write(AFWinEStrX, sx>>4);
	CoreISP3_I2C_Write(AFWinEStrY, sy>>4);
	CoreISP3_I2C_Write(AFWinEEndX, ex>>4);
	CoreISP3_I2C_Write(AFWinEEndY, ey>>4);

	CoreISP3_I2C_Write(0xE628, 0x01);	// User Area AF
	CoreISP3_Send_Command(ISP3_CMD_FULLAF);
}


static ClSint_16 System_I2C_ReadA1D1(ClUint_8 devID/**< DeviceID */, ClUint_8 addr/**< Address */)
{
     //Set_Sensor_Addr(devID>>1);
#if defined(ISP3_EVB_ENABLE)||defined(ISP2_EVB_ENABLE)
     SetReg_I2CID(devID>>1);

     return IIC_READ(addr);

#else	 
     return 0;
#endif     
}
/*
Please add below part before another access IIC.
1. I2C write: deviceID address wrdata: 0x82 0x00 0xFF
: I2C_WriteA1D1(0x82, 0x00, 0xFF)
	System_IICWrite(0x82, 0x00, 0xFF);
	
2. I2C read : deviceID address rddata: 0x83 0x00 rddata
: rddata = I2C_ReadA1D1(0x83, 0x00)

3. ISP3 write: 0xF100 <- 0xFF
: CoreISP3_I2C_Write(0xF100, 0xFF)

4. ISP3 read : 0xF100 
: a = CoreISP3_I2C_Read(0xF100)
  if(a != 0xFF) 
    return error.

5. 0xE037[7] = 0;
: a = CoreISP3_I2C_Read(0xF100)
  CoreISP3_I2C_Write(0xE037, a & 0x7F)
*/
ClUint_8 CoreISP3_IIC_Patch(void)
{
	ClUint_16 a;

	System_IICWrite(0x82, 0x00, 0xFF);  // slave id 0x82 write reg 0x00 0xff

	a = System_I2C_ReadA1D1(0x83, 0x00);   // slave id 0x83 read 0x00 reg
	  if(a != 0xFF)  
	   {
	   printk( "0x00 Write fail \n");
	    return 0;  //fail
	  	}
	
	CoreISP3_I2C_Write(0xF100, 0xFF);
	
	a = CoreISP3_I2C_Read(0xF100);
	  if(a != 0xFF)    
	   {
	   printk( "0xF100 Write fail \n");
	    return 0;  //fail
	  	}
	a = CoreISP3_I2C_Read(0xF100);
  	CoreISP3_I2C_Write(0xE037, a & 0x7F);
}

/*
1. for Touch pad
	1. 0xE037[7] = 0;
	2. ISP3 write: 0xF180 <- 0xFF 
	3. ISP3 read : 0xF180      

2. for PMU
1. BB want to access 0x81 register
	1. 0xE037[7] = 0;
	2. ISP3 write: 0xF180 <- 0xFF 
	3. ISP3 read : 0xF180 

2. BB want to access 0x83 register
	1. 0xE037[7] = 0;
	2. ISP3 I2C byte write: 
	3. ISP3 I2C byte read : 
*/
ClUint_8 CoreISP3_IIC_ForTouchPad_81Reg(void)
{
	ClUint_16 a;
	
	a= CoreISP3_I2C_Read(0xE037);
  	CoreISP3_I2C_Write(0xE037, a&0x7f);

  	CoreISP3_I2C_Write(0xf180, 0xff);
	
	a = CoreISP3_I2C_Read(0xf180);
	  if(a != 0xFF)    
	   {
	   printk( "0xf180 Write fail \n");
	    return 0;  //fail
	  	}
}

ClUint_8 CoreISP3_IIC_For83Reg(void)
{
	ClUint_16 a;
	
	a= CoreISP3_I2C_Read(0xE037);
  	CoreISP3_I2C_Write(0xE037, a&0x7f);

	System_IICWrite(0x82, 0x00, 0xFF);  // slave id 0x82 write reg 0x00 0xff

	a = System_I2C_ReadA1D1(0x83, 0x00);   // slave id 0x83 read 0x00 reg
	  if(a != 0xFF)  
	   {
	   printk( "0x00 Write fail \n");
	    return 0;  //fail
	  	}	
}



/*********************************************************************
*Name			:	CoreISP3_ControlFaceApplication
*Description	:	Set All param of Face Tracking
*Param			:	stIspFaceDetectionCtrlParam		bFaceDetectionEnable
													bFaceTrackingEnable
													bFaceAE
													bFaceAF
													bFaceAWB
													bFaceRotation
													Isp3FaceROIThick
													Isp3FaceMaxDetectionCount
													enIsp3OutResolution OutputResolution
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_ControlFaceApplication( _tIspFaceDetectionCtrl *stIspFaceDetectionCtrlParam )
{
	printk( "====> %s::%d,%d,%d,%d,%d,%d,%d,%d,%d\n", __func__, 
								stIspFaceDetectionCtrlParam->bFaceAE, 						stIspFaceDetectionCtrlParam->bFaceAF,
								stIspFaceDetectionCtrlParam->bFaceAWB, 						stIspFaceDetectionCtrlParam->bFaceDetectionEnable,
								stIspFaceDetectionCtrlParam->bFaceRotation, 				stIspFaceDetectionCtrlParam->bFaceTrackingEnable,
								stIspFaceDetectionCtrlParam->Isp3FaceMaxDetectionCount, 	stIspFaceDetectionCtrlParam->Isp3FaceROIThick,
								stIspFaceDetectionCtrlParam->OutputResolution);

	/*if((CoreISP3_I2C_Read(0xed6c)!=0x01)||(CoreISP3_I2C_Read(0xed6d)!=0x40)||(CoreISP3_I2C_Read(0xed6e)!=0x00)||(CoreISP3_I2C_Read(0xed6f)!=0x40))
	{
		printk( "Thumbnail size not 320x240\n");
		CoreISP3_I2C_Write(0xed6c, 0x01);
		CoreISP3_I2C_Write(0xed6d, 0x40);
		CoreISP3_I2C_Write(0xed6e, 0x00);
		CoreISP3_I2C_Write(0xed6f, 0x40);
	}
	*/
	
	if(stIspFaceDetectionCtrlParam->bFaceDetectionEnable == Cl_True)
	{
		CoreISP3_Send_Command(ISP3_CMD_FACE_DETECTION_START);

		/* Check each functions */
		if(stIspFaceDetectionCtrlParam->bFaceTrackingEnable)
		{
			CoreISP3_SetFaceTracking(stIspFaceDetectionCtrlParam->bFaceTrackingEnable);
		}
		if(stIspFaceDetectionCtrlParam->bFaceAE)
		{
			CoreISP3_SetFaceAE(stIspFaceDetectionCtrlParam->bFaceAE);
		}
		if(stIspFaceDetectionCtrlParam->bFaceAF)
		{
			CoreISP3_SetFaceAF(stIspFaceDetectionCtrlParam->bFaceAF);
		}
		if(stIspFaceDetectionCtrlParam->bFaceAWB)
		{
			CoreISP3_SetFaceAWB(stIspFaceDetectionCtrlParam->bFaceAWB);
		}
		if(stIspFaceDetectionCtrlParam->bFaceRotation)
		{
			CoreISP3_SetFaceRotation(stIspFaceDetectionCtrlParam->bFaceRotation);
		}
		if(stIspFaceDetectionCtrlParam->Isp3FaceROIThick>enISP_FUNC_FD_ROI_THICK_0)
		{
			CoreISP3_SetFaceROIThick(stIspFaceDetectionCtrlParam->Isp3FaceROIThick);
		}
		if(stIspFaceDetectionCtrlParam->Isp3FaceMaxDetectionCount>enISP_FUNC_FD_MAXCOUNT_1)
		{
			CoreISP3_SetFaceMaxDetectionCount(stIspFaceDetectionCtrlParam->Isp3FaceMaxDetectionCount);
		}
		if(stIspFaceDetectionCtrlParam->bFaceUserMode>enISP_FUNC_FDUser_Level_0)
		{
			CoreISP3_SetFaceUserMode(stIspFaceDetectionCtrlParam->bFaceUserMode);
		}
	}
	else
	{
		CoreISP3_Send_Command(ISP3_CMD_FACE_DETECTION_STOP);

		CoreISP3_SetFaceTracking(Cl_False);
	}
}
//Added by Jacky for Face Tracking

void CoreISP3_FaceTracking_On(void)
{
	_tIspFaceDetectionCtrl g_tIspFaceDetectCtrl;
	
	g_tIspFaceDetectCtrl.bFaceDetectionEnable		=	Cl_True;
	g_tIspFaceDetectCtrl.bFaceTrackingEnable		=	Cl_True;
	g_tIspFaceDetectCtrl.bFaceAE					=	Cl_False;  // enISP_FUNC_AEwithFACE_1; // Jacky change this
	g_tIspFaceDetectCtrl.bFaceAF					=	Cl_False;
	g_tIspFaceDetectCtrl.bFaceAWB					=	Cl_False;
	g_tIspFaceDetectCtrl.bFaceRotation				=	Cl_True;
	g_tIspFaceDetectCtrl.Isp3FaceROIThick			=	enISP_FUNC_FD_ROI_THICK_4;
	g_tIspFaceDetectCtrl.Isp3FaceMaxDetectionCount	=	enISP_FUNC_FD_MAXCOUNT_10;   // enISP_FUNC_FD_MAXCOUNT_10;  Jacky change this for test
	g_tIspFaceDetectCtrl.bFaceUserMode 				=	enISP_FUNC_FDUser_Level_5;   // Jacky change user mode
		
	CoreISP3_ControlFaceApplication(&g_tIspFaceDetectCtrl);
}

void CoreISP3_FaceTracking_Off(void)
{
	_tIspFaceDetectionCtrl g_tIspFaceDetectCtrl;
	
	g_tIspFaceDetectCtrl.bFaceDetectionEnable	= Cl_False;
		
	CoreISP3_ControlFaceApplication(&g_tIspFaceDetectCtrl);
}

/*
0x001 [5:0] ro 0x000 DNUM Face Detection Result Count Register 
Permitted values 0x0 ~ 0x23(Max 35) 
*/
void face_Reg_Read_Set(short faceAddrH, short faceAddrL, ClUint_8 *read_dataL, ClUint_8 *read_dataH)
{
	printk( " ====> %s:: 0x%x\n", __func__, faceAddrH<<8|faceAddrL);
	CoreISP3_I2C_Write(FaceAddrH,(short)faceAddrH);
	CoreISP3_I2C_Write(FaceAddrL,(short)faceAddrL);
	CoreISP3_I2C_Write(FaceRW,(short)0x01);
	CoreISP3_I2C_Write(FaceRW,(short)0x00);
	*read_dataL = (int)CoreISP3_I2C_Read(FaceRDataL);
	*read_dataH = (int)CoreISP3_I2C_Read(FaceRDataH);
}


ClUint_32 CoreISP3_ReadFace_Cnt(void)
{
	ClUint_8 read_value, read_dataL, read_dataH;
	ClUint_16 face_cnt;
/*
	face_Reg_Read_Set((short)0x00, (short)0x01, &read_dataL, &read_dataH);
	face_cnt = CoreISP3_I2C_Read(FaceRDataL);
	printk( " ====> %s:: FaceRDataL: %d\n", __func__, face_cnt);
*/
	
	face_cnt = CoreISP3_I2C_Read(FaceResultCnt);
	printk( " ====> %s:: FaceResultCnt: %d\n", __func__, face_cnt);

	return face_cnt;
}

void CoreISP3_ReadFace_AFPosition(void)
{
	ClUint_16 face_af;
	
	face_af = CoreISP3_I2C_Read(FaceAFPosH)<<8|CoreISP3_I2C_Read(FaceAFPosL);
	
	printk( " ====> %s::F : %d\n", __func__, face_af);

}

void CoreISP3_ReadFaceA_Position(void)
{
	ClUint_16 face_a_strx;
	ClUint_16 face_a_stry;
	ClUint_16 face_a_endx;
	ClUint_16 face_a_endy;
	
	face_a_strx = CoreISP3_I2C_Read(FaceAStrX_8051H)<<8|CoreISP3_I2C_Read(FaceAStrX_8051L);
	face_a_stry = CoreISP3_I2C_Read(FaceAStrY_8051H)<<8|CoreISP3_I2C_Read(FaceAStrY_8051L);
	face_a_endx = CoreISP3_I2C_Read(FaceAEndX_8051H)<<8|CoreISP3_I2C_Read(FaceAEndX_8051L);
	face_a_endy = CoreISP3_I2C_Read(FaceAEndY_8051H)<<8|CoreISP3_I2C_Read(FaceAEndY_8051L);
	#if 1
       //  (1280x960)/4 = (320x240)
	_faceInfo[0].sx = face_a_strx/4;
	_faceInfo[0].sy = face_a_stry/4;
	_faceInfo[0].ex = face_a_endx/4;
	_faceInfo[0].ey = face_a_endy/4;
	#else
       //  (1280x960) -> (240x180)
	_faceInfo[0].sx = face_a_strx*(1280/240);
	_faceInfo[0].sy = face_a_stry*(1280/240);
	_faceInfo[0].ex = face_a_endx*(1280/240);
	_faceInfo[0].ey = face_a_endy*(1280/240);
	#endif
	printk( " ====> %s:: [%d %d %d %d]\n", __func__, _faceInfo[0].sx, _faceInfo[0].sy, _faceInfo[0].ex, _faceInfo[0].ey);

}

/*********************************************************************
*Name			:	CoreISP3_ReadJPEGRowCnt
*Description	:	  
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
ClUint_16 CoreISP3_ReadJPEGRowCnt( void )
{
    ClUint_16 RowCnt;
	
    RowCnt = CoreISP3_I2C_Read(RowCntH)<<8|CoreISP3_I2C_Read(RowCntL);
		
    if((RowCnt<480)||(RowCnt>1200))
    	{
	    printk( "Row cnt over flow %d\n",  RowCnt);
	    RowCnt = 1200;    // Jacky for protect
    	}
    printk( "\n read fixed jpeg stream size 2048 x %d \n",  RowCnt);

    return RowCnt;
}


ClUint_32 CoreISP3_ReadJPEG_Size(void)
{
ClUint_8 overflow_flag;
ClUint_16 size_h;
ClUint_16 size_m;
ClUint_16 size_l;
/*
0xee01   [7:7] r 0x00 Overflow JPEG Fifo Overflow Check Flag 0xee01 
		[6:0] r 0x00 JSizeH JPEG Size High 
0xee02   [7:0] r 0x00 JSizeM JPEG Size Medium 
0xee03   [7:0] r 0x00 JSizeL JPEG Size Low 
*/

size_h = CoreISP3_I2C_Read(0xee01);
size_m = CoreISP3_I2C_Read(0xee02);
size_l = CoreISP3_I2C_Read(0xee03);

overflow_flag= (size_h&0x80)>>7;

printk( "\n Read jpeg size [%d] [%d] [%d] [%d] \n", size_h, size_m, size_l, ((size_h&0x7f)<<16)|(size_m<<8)|(size_l));

return ((size_h&0x7f)<<16)|(size_m<<8)|(size_l); 
}

//0xee15 [7:0] rw 0x18 RSF JPEG Quality Factor(18:Good <-> 28:Poor) 
void CoreISP3_SetJPEG_Quality_Factor(enIsp3JPEGQF value)
{
    printk( "====> %s::%d\n", __func__, value);
	
    CoreISP3_I2C_Write(0xee15, value);
}

/*********************************************************************
*Name			:	CoreISP3_OutpJPEG_Resolution
*Description	:		Set ISP Mode 
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_OutpJPEG_Resolution( Cl_Bool bThumbnail , enIsp3OutResolution OutResolution)
{
    printk( "====> %s::%d\n", __func__, bThumbnail);

    /* Global Variable Set */
#ifdef ISP3_EVB_ENABLE
    g_bJpegMode = ClTrue;
#endif
/*
default is 0x200=512            0x800=2048
0xee17 [7:0] rw 0x02 JPGFifoSizeH JPEG FIFO Limit control High 
0xee18 [7:0] rw 0x00 JPGFifoSizeL JPEG FIFO Limit control Low 

JPEG Output row count    0x2cc is original value

2. FIFO size x Hync count.
   FIFO size is 2048. This is fixed.
   
3. If we want to use 1280 x 960, our ISP3 will make a 2048 x 960 because FIFO size is 2048 fix.
   Thus, 2048 x 960 = around 1.9M. BB need to allocation around 1.9MByte.
   
4. Ex) 2048 x 960 resolution
             0xEe, 0x0a, 0x03, //   #RowCntH
             0xEe, 0x0b, 0xc0, //   #RowCntL
   Ex) 2048 x 480 resolution
             0xEe, 0x0a, 0x01, //   #RowCntH
             0xEe, 0x0b, 0xE0, //   #RowCntL

*/


#ifdef __ISP3_R__  
    CoreISP3_I2C_Write(JpgOutMode, 0x18);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 
    //CoreISP3_I2C_Write(JpgOutMode, 0x16);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 
    //CoreISP3_I2C_Write(JpgOutMode, 0x12);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 

#else //__ISP3_R__
    CoreISP3_I2C_Write(JpgOutMode, 0x08);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 
    CoreISP3_I2C_Write(Jpg_FifoLimit_H, 0x01);   //#0: 288, 1: 192, 2: 96, 3: JPEG fifo size high 5bit, low 8bit, else: 192 
    CoreISP3_I2C_Write(Jpg_FifoLimit_L, 0xc0);   //#JPEG fifo size low byte
#endif //__ISP3_R__

/* Jpeg data + padding data size */
#ifdef __ISP3_R__    // Jacky add this for test
    printk("====> jpeg  output OutResolution \n", OutResolution);
    printk("jpg fifoH: 0x%x   L:0x%x \n", CoreISP3_I2C_Read(Jpg_FifoLimit_H), CoreISP3_I2C_Read(Jpg_FifoLimit_L));
    if(OutResolution == enISP_RES_5MP_FULL)   // 2560x1920
    	{  // data size: 2048x1920
	    CoreISP3_I2C_Write(RowCntH, 0x07);   //JPEG Output row count High
	    CoreISP3_I2C_Write(RowCntL, 0x80);   //JPEG Output row count Low
    	}
    else if(OutResolution == enISP_RES_QXGA)  // 2048x1536
    	{  // data size: 2048x1536
	    CoreISP3_I2C_Write(RowCntH, 0x06);   //JPEG Output row count High
	    CoreISP3_I2C_Write(RowCntL, 0x00);   //JPEG Output row count Low
    	}   
    else if(OutResolution == enISP_RES_UXGA)  // 1600x1200
    	{  // data size: 2048x1200
	    CoreISP3_I2C_Write(RowCntH, 0x04);   //JPEG Output row count High
	    CoreISP3_I2C_Write(RowCntL, 0xb0);   //JPEG Output row count Low
    	}   
    else if(OutResolution == enISP_RES_1_3MP)  // 1280x960
    	{  // data size: 2048x960
	    CoreISP3_I2C_Write(RowCntH, 0x03);   //JPEG Output row count High
	    CoreISP3_I2C_Write(RowCntL, 0xc0);   //JPEG Output row count Low
    	}   
    else if(OutResolution == enISP_RES_SVGA)  // 800x600
    	{  // data size: 2048x600
	    CoreISP3_I2C_Write(RowCntH, 0x02);   //JPEG Output row count High
	    CoreISP3_I2C_Write(RowCntL, 0x58);   //JPEG Output row count Low
    	}   
    else if(OutResolution == enISP_RES_VGA)  // 640x480
    	{// data size: 2048x480 = 983040bytes            // jade read size: 2048*(480+6) = 995733   1018333(JHBlank=0)       Jade get size is fixed 1136703 
	    CoreISP3_I2C_Write(RowCntH, 0x01);   //JPEG Output row count High
	    CoreISP3_I2C_Write(RowCntL, 0xe0);   //JPEG Output row count Low
    	}   
   else // 716
    	{
	    CoreISP3_I2C_Write(RowCntH, 0x02);   //JPEG Output row count High
	    CoreISP3_I2C_Write(RowCntL, 0xcc);   //JPEG Output row count Low
    	}
#else //__ISP3_R__
    CoreISP3_I2C_Write(RowCntH, 0x03);   //JPEG Output row count High
    CoreISP3_I2C_Write(RowCntL, 0xcc);   //JPEG Output row count Low
#endif //__ISP3_R__

    CoreISP3_I2C_Write(FlipHold, 0xff);   //JPEG Rotation (8x8) Parameter
    
    CoreISP3_I2C_Write(JHBlank, 0x06);   //JPEG Output Horizontal Blank Size
    //CoreISP3_I2C_Write(Still_UpHBlankL, 0x10);   //Up scaling line blank
    
    //CoreISP3_I2C_Write(JHBlank, 0x00);   //JPEG Output Horizontal Blank Size   Jacky change for test
    CoreISP3_I2C_Write(Still_UpHBlankL, 0x10);   //Up scaling line blank
    
    CoreISP3_I2C_Write(Still_VRdRate_SclMode, 0x0f);   //JPEG Scaling mode[auto mode : 0x0F, average : 0x0A, linear : 0x05, subsample : 0x00]
    CoreISP3_I2C_Write(Thu_VRdRate_SclMode, 0x02);  //Thumbnail Scaling mode[auto mode : 0x0F,  average : 0x0A, linear : 0x05, subsample : 0x00]
    CoreISP3_I2C_Write(Pre_VRdRate_SclMode, 0x0f);   //Preview Scaling mode[auto mode : 0x0F,  average : 0x0A, linear : 0x05, subsample : 0x00]

    if(bThumbnail)
    {   
#ifdef __ISP3_R__
        /*JPCLKDiv = JpgClkDiv[3:0] PrvCLKDiv = JpgClkDiv[3:0] */
        CoreISP3_I2C_Write(JpgClkDiv, 0x02);   //PrvCLKDiv (2 = JClck * 1/2)
        /* Face Detection OFF */
        CoreISP3_I2C_Write(FaceModeReg, 0x00);   //Face off
#else   //__ISP3_R__
        /*JPCLKDiv = JpgClkDiv[3:0] PrvCLKDiv = JpgClkDiv[3:0] */
        CoreISP3_I2C_Write(JpgClkDiv, 0x01);   //PrvCLKDiv
#endif //__ISP3_R__        
    
        CoreISP3_I2C_Write(PrvMode, 0x01);   //Preview Mode On
        WaitTime_us(2*100);    // 100ms
    }

    /* OUT FORMAT */
    CoreISP3_I2C_Write(OutFmt_JPG, 0x83);   //[0:0] OutFmt ' 0' = YCbCr, '1' = JPEG      [1:1] rw 0x00 JPGENCODE JPEG encoder enable  [0:0] rw 0x00 JPGOUTEN JPEG mode output enable 
    WaitTime_us(2*100);    // 100ms
    
    /* Send Command(Output Jpeg Mode) to ISP */
    CoreISP3_Send_Command(ISP3_CMD_CAPTURE);       
    WaitTime_us(2*500);    // 500ms
#ifdef DOWN_TIME_CHECK
    printk( "Isp3 200ms time check start: %d\n", TIMER_curTime_ms(1));
    //WaitTime_us(2*200);    // 500ms
    printk( "Isp3 200ms time check end: %d\n", TIMER_curTime_ms(1));
#endif
    //general_ctrl.output_mode = 1;
    //printk("====> %s, %d\n", __func__, general_ctrl.output_mode);
}

/*********************************************************************
*Name			:	CoreISP3_OutpJPEG
*Description	:	Set ISP Mode 
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_OutpJPEG( Cl_Bool bThumbnail )
{
    printk( "====> %s::%d\n", __func__, bThumbnail);
/*
										On		Off
Thumbnail		PrvMode		(0xEE1A)		0x01	0x00 
Dummy			JpgOutMode	(0xEE16)		0x28	0x26
*/
    /* Global Variable Set */
#ifdef ISP3_EVB_ENABLE
    g_bJpegMode = ClTrue;
#endif

#ifdef __ISP3_R__  
    CoreISP3_I2C_Write(JpgOutMode, 0x18);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 

#else //__ISP3_R__
    CoreISP3_I2C_Write(JpgOutMode, 0x08);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 
    CoreISP3_I2C_Write(Jpg_FifoLimit_H, 0x01);   //#0: 288, 1: 192, 2: 96, 3: JPEG fifo size high 5bit, low 8bit, else: 192 
    CoreISP3_I2C_Write(Jpg_FifoLimit_L, 0xc0);   //#JPEG fifo size low byte
#endif //__ISP3_R__

/* Jpeg data + padding data size */
#ifdef __ISP3_R__  
    //240
    //CoreISP3_I2C_Write(RowCntH, 0x00);   //JPEG Output row count High
    //CoreISP3_I2C_Write(RowCntL, 0xf0);   //JPEG Output row count Low
	//480
   // printk( "\n jpeg stream size 2048x480 \n");  
    CoreISP3_I2C_Write(RowCntH, 0x01);   //JPEG Output row count High
    CoreISP3_I2C_Write(RowCntL, 0xe0);   //JPEG Output row count Low
    printk( "\n fixed jpeg stream size 2048 x %d \n", CoreISP3_I2C_Read(RowCntH)<<8|CoreISP3_I2C_Read(RowCntL) );
    //716
    //CoreISP3_I2C_Write(RowCntH, 0x02);   //JPEG Output row count High
    //CoreISP3_I2C_Write(RowCntL, 0xcc);   //JPEG Output row count Low
#else //__ISP3_R__
    CoreISP3_I2C_Write(RowCntH, 0x03);   //JPEG Output row count High
    CoreISP3_I2C_Write(RowCntL, 0xcc);   //JPEG Output row count Low
#endif //__ISP3_R__

    CoreISP3_I2C_Write(FlipHold, 0xff);   //JPEG Rotation (8x8) Parameter
    
    CoreISP3_I2C_Write(JHBlank, 0x06);   //JPEG Output Horizontal Blank Size
    CoreISP3_I2C_Write(Still_UpHBlankL, 0x10);   //Up scaling line blank
    
    CoreISP3_I2C_Write(Still_VRdRate_SclMode, 0x0f);   //JPEG Scaling mode[auto mode : 0x0F, average : 0x0A, linear : 0x05, subsample : 0x00]
    CoreISP3_I2C_Write(Thu_VRdRate_SclMode, 0x02);  //Thumbnail Scaling mode[auto mode : 0x0F,  average : 0x0A, linear : 0x05, subsample : 0x00]
    CoreISP3_I2C_Write(Pre_VRdRate_SclMode, 0x0f);   //Preview Scaling mode[auto mode : 0x0F,  average : 0x0A, linear : 0x05, subsample : 0x00]

    if(bThumbnail)
    {   
#ifdef ISP3_EVB_ENABLE
    	  g_bJpegThumbnail = Cl_True;
#endif //

#ifdef __ISP3_R__
        /*JPCLKDiv = JpgClkDiv[3:0] PrvCLKDiv = JpgClkDiv[3:0] */
        CoreISP3_I2C_Write(JpgClkDiv, 0x02);   //PrvCLKDiv (2 = JClck * 1/2)
        /* Face Detection OFF */
        CoreISP3_I2C_Write(FaceModeReg, 0x00);   //Face off
#else   //__ISP3_R__
        /*JPCLKDiv = JpgClkDiv[3:0] PrvCLKDiv = JpgClkDiv[3:0] */
        CoreISP3_I2C_Write(JpgClkDiv, 0x01);   //PrvCLKDiv
#endif //__ISP3_R__        
    
        CoreISP3_I2C_Write(PrvMode, 0x01);   //Thumbnail On
        WaitTime_us(2*100);    // 100ms
    }
   else
   	{
#ifdef ISP3_EVB_ENABLE
    	  g_bJpegThumbnail = Cl_False;
#endif //
   	  // no dummy, no thumbnail.. already test ok, after ff d9 still have some data
    	  CoreISP3_I2C_Write(JpgOutMode, 0x16);   //JpgClk Div [5:4] 0:x1 1:x1/2  2:x1/4  [3:0] 6: No Padding data 8:Padding data 
    	
        CoreISP3_I2C_Write(PrvMode, 0x00);   //Thumbnail Off
        WaitTime_us(2*100);    // 100ms
   	}
    /* OUT FORMAT */
    CoreISP3_I2C_Write(OutFmt_JPG, 0x83);   //[0:0] OutFmt ' 0' = YCbCr, '1' = JPEG
    WaitTime_us(2*100);    // 100ms
    
    /* Send Command(Output Jpeg Mode) to ISP */
    //CoreISP3_Send_Command(ISP3_CMD_CAPTURE);         // saving time
    //WaitTime_us(2*500);    // 500ms

	
    printk("jpg fifoH: 0x%x   L:0x%x \n", CoreISP3_I2C_Read(Jpg_FifoLimit_H), CoreISP3_I2C_Read(Jpg_FifoLimit_L));
    CoreISP3_ReadJPEG_Size();

}
/*********************************************************************
*Name			:	CoreISP3_OutpYCbCr
*Description	:	Set ISP Mode
*Param			:	none
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_OutpYCbCr( void )
{
	printk( "====> %s\n", __func__);
	 /* Global Variable Set */
#ifdef ISP3_EVB_ENABLE
    g_bJpegMode = Cl_False;
#endif
    /* OUT FORMAT */
    CoreISP3_I2C_Write(OutFmt_JPG, 0x82);   //[0:0] OutFmt ' 0' = YCbCr, '1' = JPEG
    WaitTime_us(2*100);    // 100ms

    CoreISP3_Send_Command(ISP3_CMD_PREVIEW);
    WaitTime_us(2*500);    // 500ms
}

void CoreISP3_RGB565_Out(void)
{
	ClUint_8 reg_data = 0;
	//reg_data = CoreISP3_I2C_Read(OutFmt); 
	//CoreISP3_I2C_Write(OutFmt, (reg_data | 0x10));
	//DebugMessage(DM_JPEG, "reg_data 0x%x\n", reg_data);
	CoreISP3_I2C_Write(OutFmt, 0x1a);
}
/*********************************************************************
*Name			:	CoreISP3_SystemControl
*Description	:	Set ISP Sleep & Wake Up mode
*Param			:	param		ON : Sleep Mode
								OFF : Wake Up Mode
*return			:	none
*Author			:	
*Remark			:	
*Log			:	
**********************************************************************/
void CoreISP3_SystemControl( ClUint_16 param )
{
       ClUint_32    temp;	
       
       	printk( "====> %s::%d\n", __func__, param);
		printk( "System contron on \n");


		CoreISP3_I2C_Write(0xe010, 0xe0); //4 [7] : Sensor reset low, vcm disable    Reset pin low for sensor sleep    .. useless

		//CoreISP3_I2C_Write(0xEDE0, 0x00);  //useless
		
		CoreISP3_I2C_Write(0xE058, /*0x3f*/0x30);
		WaitTime_us(2*100);     //4 100 ms
		   
		CoreISP3_I2C_Write(0xE052, /*0xff*/0x00);
		WaitTime_us(2*100);     //4 100 ms
		   
		temp = CoreISP3_I2C_Read(0xE0A2);
		CoreISP3_I2C_Write(0xE0A2, temp & 0xEF);
		printk( "\nCoreISP3_I2C_Read(0xE0A2)::0x%x\n", CoreISP3_I2C_Read(0xE0A2));
		WaitTime_us(2*100);     //4 100 ms

		CoreISP3_I2C_Write(0xe060, /*0x03*/0x02); //4 PLL Register
		WaitTime_us(2*100);    //4 100 ms

		//temp = CLKEN;            //bit17        0010 0000 0000 0000 0000
		//CLKEN = (temp&0xfffdffff);          // Add this line 0.8mA 
		
		//ExtGPIO_WriteData(1, 1);      //Vddc  1.2V off    only this line  1.3 mA sleep current  

		// Sensor Reset Pin
		//SI2CCNTL = 1;
		//SPSC = 1	;
		//DUALSCNTL=0;
}

void CoreISP3_SetSensorInfo( enIsp3OutResolution Isp3OutResolution )
{
#ifdef ISP3_EVB_ENABLE
   // tSensorInfo g_Isp3SensorInfo;
    
    ////DebugMessage(DM_ISP, "CoreISP3_SetSensorInfo %d\n", Isp3OutResolution);

    switch(Isp3OutResolution)
    {
        case enISP_RES_5MP_FULL:    // 2560 x 1920
            g_Isp3SensorInfo.width=2560;
            g_Isp3SensorInfo.height=1920;
            break;
        case enISP_RES_QXGA:    // 2048 x 1536
            g_Isp3SensorInfo.width=2048;
            g_Isp3SensorInfo.height=1536;
            break;            
        case enISP_RES_UXGA:     // 1600 x 1200
            g_Isp3SensorInfo.width=1600;
            g_Isp3SensorInfo.height=1200;
            break;
        case enISP_RES_SXGA:    // 1280 x 1024
            g_Isp3SensorInfo.width=1280;
            g_Isp3SensorInfo.height=1024;        
            break;
        case enISP_RES_1_3MP:   // 1280 x 960
            g_Isp3SensorInfo.width=1280;
            g_Isp3SensorInfo.height=960;
            break;
        case enISP_RES_XGA:     // 1024 x  768
            g_Isp3SensorInfo.width=1024;
            g_Isp3SensorInfo.height=768;
            break;
        case enISP_RES_SVGA:    // 800 x  600
            g_Isp3SensorInfo.width=800;
            g_Isp3SensorInfo.height=600;
            break;
        case enISP_RES_VGA:     // 640 x  480
            g_Isp3SensorInfo.width=640;
            g_Isp3SensorInfo.height=480;
            break;
        case enISP_RES_QVGA:    // 320 x  240
            g_Isp3SensorInfo.width=320;
            g_Isp3SensorInfo.height=240;
            break;
        case enISP_RES_HVGA:    // 320 x  240
            g_Isp3SensorInfo.width=480;
            g_Isp3SensorInfo.height=320;
            break;
	case enISP_RES_480_360:  // Added by Jacky
            g_Isp3SensorInfo.width=480;
            g_Isp3SensorInfo.height=360;
            break;
	case enISP_RES_480_320:  // Added by Jacky
            g_Isp3SensorInfo.width=480;
            g_Isp3SensorInfo.height=320;
            break;
        case enISP_RES_800_480:    // 
            g_Isp3SensorInfo.width=800;
            g_Isp3SensorInfo.height=480;
            break;
        case enISP_RES_1600_960:    // 
            g_Isp3SensorInfo.width=1600;
            g_Isp3SensorInfo.height=960;
            break;
        case enISP_RES_2048_1228:    // 
            g_Isp3SensorInfo.width=2048;
            g_Isp3SensorInfo.height=1228;
            break;
        case enISP_RES_2560_1536:    // 
            g_Isp3SensorInfo.width=2560;
            g_Isp3SensorInfo.height=1536;
            break;
        default:
            break;
    }
    g_Isp3SensorInfo.fullwidth=2560;   // 500M
    g_Isp3SensorInfo.fullheight=1920;  
	
   printk( "Resolution setting change, sensor: [%d x %d] \n", g_Isp3SensorInfo.width, g_Isp3SensorInfo.height);
   OSAL_MemCopy(&g_stSensorInfo, &g_Isp3SensorInfo, sizeof(g_Isp3SensorInfo));  
#else
	printk( "Resolution setting change \n");
#endif	
}

void CoreISP3_PLLOn(Cl_Bool bOn)
{
	//bPLL = bOn;
	if(bOn)
	{
		CoreISP3_I2C_Write(0xE060, 0x00);
	}
	else
	{
		CoreISP3_I2C_Write(0xE060, 0x03);
	}
}

void CoreISP3_FlickerSuppression(FLICKER_TYPE type)
{
    	printk( "\n %s %d \n",__func__, type);
		
#if 0  // old 	
	switch(type)
	{
		case FLICKER_OFF:
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_OFF);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_OFF);
			break;

		case FLICKER_AUTO:
			CoreISP3_I2C_Write(0xE628, 0x01);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_ON);
			break;
			
		case FLICKER_50HZ:
			CoreISP3_I2C_Write(0xE628, 0x32);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_OFF);
			break;
			
		case FLICKER_60HZ:
			CoreISP3_I2C_Write(0xE628, 0x3c);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
			CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_OFF);
			break;
	}
#else  // update latest 
		switch(type)
		{
			case FLICKER_50HZ_OFF:  // 0
				CoreISP3_I2C_Write(0xE628, 0x01);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_OFF);
				break;

			case FLICKER_50HZ_ON:
				CoreISP3_I2C_Write(0xE628, 0x01);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
				break;
				
			case FLICKER_60HZ_OFF:
				CoreISP3_I2C_Write(0xE628, 0x3C);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_OFF);
				break;
				
			case FLICKER_60HZ_ON:
				CoreISP3_I2C_Write(0xE628, 0x3C);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
				break;
				
			case FLICKER_AUTO_OFF:
				CoreISP3_I2C_Write(0xE628, 0x01);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_OFF);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_OFF);
				break;
				
			case FLICKER_AUTO_ON:
				CoreISP3_I2C_Write(0xE628, 0x01);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_SUPPRESSION_ON);
				CoreISP3_Send_Command(ISP3_CMD_FLICKER_DETECTION_ON);
				break;
		}

#endif
}

void CoreISP3_AE_SetEV(int ev_Y/**< EV : -5~+5, 0:Default, -5: Dark, +5:Bright */)
{
	CoreISP3_I2C_Write(0xE62A, ev_Y);
	CoreISP3_Send_Command(ISP3_CMD_SET_AE_CONFIG);
}

void CoreISP3_AE_SetISO(enIsp3AEISO isoLevel/**< ISO Level : 1~8, 0: Auto 1: Low ISO, 8: High ISO*/)
{
    	printk( "\n %s %d \n",__func__, isoLevel);
		
	switch(isoLevel)
	{
		case enISP_AE_ISO_AUTO:	// AUTO
			CoreISP3_I2C_Write(0xE65A, 0x00);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_100: // ISO 100
			CoreISP3_I2C_Write(0xE65A, 0x01);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_200:// ISO 200
			CoreISP3_I2C_Write(0xE65A, 0x02);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_300:// ISO 300
			CoreISP3_I2C_Write(0xE65A, 0x03);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_400:// ISO 400
			CoreISP3_I2C_Write(0xE65A, 0x04);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_500:// ISO 500
			CoreISP3_I2C_Write(0xE65A, 0x05);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_600:// ISO 600
			CoreISP3_I2C_Write(0xE65A, 0x06);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_700:// ISO 700
			CoreISP3_I2C_Write(0xE65A, 0x07);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		case enISP_AE_ISO_800:// ISO 800
			CoreISP3_I2C_Write(0xE65A, 0x08);
			CoreISP3_Send_Command(ISP3_CMD_AE_SET_ISO);
			break;
			
		default:
			break;
	}

	CoreISP3_GetISOGain();  // Jacky for test
}
//float
ClUint_32 CoreISP3_Get_Version(void)
{
	ClUint_32 SW_Ver;
	ClUint_16 majorVer;
	ClUint_16 minorVer;
	ClUint_16 data1;
	ClUint_16 data0;
	
	CoreISP3_Send_Command(ISP3_CMD_GET_VERSION);  // Jacky change this

	data1  = CoreISP3_I2C_Read(0xE628);
	data0  = CoreISP3_I2C_Read(0xE629);
	majorVer   = data1<<8 | data0;

	data1 = CoreISP3_I2C_Read(0xE62A);
	data0 = CoreISP3_I2C_Read(0xE62B);
	minorVer  = data1<<8 | data0;

	SW_Ver = majorVer <<16 | minorVer;

    	printk( " ====Get ISP vision  %d [0x%x]====\n", SW_Ver, SW_Ver);
	printk(" ====Get ISP vision  %d [0x%x]====\n", SW_Ver, SW_Ver);
	return SW_Ver;

}

void CoreISP3_SetAWBMode(enWB_MANUAL_TYPE type)
{
    	printk( " %s %d \n",__func__, type);
	CoreISP3_I2C_Write(0xEb98, (ClUint_8)type);
}
				

// Added by Jacky according ISP3R function flow
void CoreISP3_SetSceneMode(enIsp3SceneMode mode)
{
    	 printk( " %s %d \n",__func__, mode);
		 
	 CoreISP3_I2C_Write(0xE628, mode);
	 CoreISP3_Send_Command(ISP3_CMD_MODE_AUTOMODE);  // Jacky change this   0x20
}

/* =============================================================================================================
Description :	UnSet WDR Mode
============================================================================================================= */
void CoreISP3_Set_WDR_Off(void)
{
	_tISP_WDR_CTRL IspWdrCtrl;

	IspWdrCtrl.bWDREnable = Cl_False;
	CoreISP3_SetWDR(&IspWdrCtrl);
}


void CoreISP3_Set_StillStabilizer_On(void)
{
	_tIspStillStabilizerCtrl  StillStabilizerParam;
	StillStabilizerParam.bStillStabilizerEnable = Cl_True;
	StillStabilizerParam.StillStabilzerLevel = enISP_FUNC_SS_LEVEL_1_2;
	CoreISP3_SetStillStabilizer(&StillStabilizerParam);
}


void CoreISP3_Set_StillStabilizer_Off(void)
{
	_tIspStillStabilizerCtrl  StillStabilizerParam;
	StillStabilizerParam.bStillStabilizerEnable = Cl_False;
	StillStabilizerParam.StillStabilzerLevel = enISP_FUNC_SS_LEVEL_1_2;
	CoreISP3_SetStillStabilizer(&StillStabilizerParam);
}

#define STEP_GAP  12  // 5

/*
0xE002

[7:0] rw 0x00 ISPFENB ISP function enable set B 0xe002 
[7:7] rw 0x00 ENHUE Hue Control 
[6:6] rw 0x00 ENSAT Saturation Control 
[5:5] rw 0x00 ENGRY Enable gray effect 
[4:4] rw 0x00 ENBPC Enable black offset adjustment 
[3:3] rw 0x00 ENBCC Enable brightness & contrast control  
[2:2] rw 0x00 ENATC Enable adaptive tone curve selection with fuzzy algori
8051 code 
[1:1] rw 0x00 ENRTNX Enable retinex effect for boosting low light area
preserving high light area tone gradation 
[0:0] rw 0x00 ENFCSHL Enable false color suppression in high & low light area 

*/
void CoreISP3_SetBrightness(enIsp3Level_Value val)
{
    	 printk( " %s %d ->[0x%x] \n", __func__, val, (0x40+((int)val-5)*STEP_GAP));
		 
	 CoreISP3_I2C_Write(0xE5E1, (ClUint_8)(0x40+((int)val-(enISP_Level_Max-1)/2)*STEP_GAP));	 
	 CoreISP3_I2C_Partial_Write(0xE002, 2, 2, 1); // brightness enable
}
 
void CoreISP3_SetContrast(enIsp3Level_Value val)
{
    	 printk( " %s %d ->[0x%x] \n", __func__, val, ((0x00+((int)val)*STEP_GAP)&0xFF));
		 
	 CoreISP3_I2C_Write(0xE5E0, (ClUint_8)((0x00+((int)val)*STEP_GAP)&0xFF));
	 CoreISP3_I2C_Partial_Write(0xE002, 2, 2, 1); // contrast enable
}
 
void CoreISP3_SetSaturation(enIsp3Level_Value val)  // default 0x40
{
    	 printk( " %s %d ->[0x%x] \n", __func__, val, (0x80+((int)val-5)*STEP_GAP));
		 
	 CoreISP3_I2C_Write(0xE5B0, (ClUint_8)(0x80+((int)val-(enISP_Level_Max-1)/2)*STEP_GAP));
	 CoreISP3_I2C_Partial_Write(0xE002, 6, 6, 1); // satuation enable
}
 
void CoreISP3_SetHue(enIsp3Level_Value val)
{
    	 printk( " %s %d ->[0x%x] \n", __func__, val, ((0x00+((int)val)*STEP_GAP)&0xFF));
		 
	 CoreISP3_I2C_Write(0xE5B1, (ClUint_8)((0x00+((int)val)*STEP_GAP)&0xFF));
	 CoreISP3_I2C_Partial_Write(0xE002, 7, 7, 1); //  hue enable
}

void CoreISP3_Brightness_OnOff(Cl_Bool Brightness)
{
#ifdef ISP3_EVB_ENABLE
	printk( "====> %s::%d\n", __func__, Brightness);
#endif

   if(Brightness)
    {
	//# Level (Default =0x40) 0xE5E1[7:0] = 10step(gap = 0x05)
	CoreISP3_I2C_Write(0xE5E1, 0x40);
	//# Enable = 0xE002[2:2] = On(1)
	CoreISP3_I2C_Write(0xE002, 1<<2);		
    }
    else
    {
        //# Disable = 0xE002[2:2] = Off (0)
        CoreISP3_I2C_Write(0xE002, 0<<2);	
    }
}

void CoreISP3_Contrast_OnOff(Cl_Bool Contrast)
{
#ifdef ISP3_EVB_ENABLE
	printk( "====> %s::%d\n", __func__, Contrast);
#endif
   if(Contrast)
    {
		//# Level (Default =0x00) 0xE5E0[7:0] = 10step(gap = 0x05)
		CoreISP3_I2C_Write(0xE5E0, 0x00);
		//# Enable = 0xE002[2:2] = On(1)
		CoreISP3_I2C_Write(0xE002, 1<<2);		
    }
    else
    {
        //# Disable = 0xE002[2:2] = On(1)
        CoreISP3_I2C_Write(0xE002, 0<<2);	
    }
}

void CoreISP3_Saturation_OnOff(Cl_Bool Saturation)
{
#ifdef ISP3_EVB_ENABLE
	printk( "====> %s::%d\n", __func__, Saturation);
#endif
	if(Saturation)
    {
		//# Level (Default =0x80) 0xE5B0 [7:0] = 10step(gap = 0x05)
		CoreISP3_I2C_Write(0xE5B0, 0x80);
		//# Enable = 0xE002[6:6] = On(1)
		CoreISP3_I2C_Write(0xE002, 1<<6);		
    }
    else
    {
        //# Enable = 0xE002[6:6] = Off (0)
        CoreISP3_I2C_Write(0xE002, 0<<6);	
    }
}

void CoreISP3_Hue_OnOff(Cl_Bool Hue)
{
#ifdef ISP3_EVB_ENABLE
	printk( "====> %s::%d\n", __func__, Hue);
#endif
   if(Hue)
    {
		//# Level (Default =0x00) 0xE5B1[7:0] = 10step(gap = 0x05)
		CoreISP3_I2C_Write(0xE5B1, 0x00);
		//# Enable = 0xE002[7:7] = On(1)
		CoreISP3_I2C_Write(0xE002, 1<<7);		
    }
    else
    {
        //# Disable = 0xE002[7:7] = Off (0)
        CoreISP3_I2C_Write(0xE002, 0<<7);	
    }
}

//  // Jacky update 
// We can change YUV swap as below.
void CoreISP3_YUV_Swap(enIsp3YUV_Swap mode)
{
/*===========================================================
    enISP_CrYCb    = 0x08,   	
    enISP_YCrCb    = 0x09,     	
    enISP_CbYCr    = 0x0a,  
    enISP_YCbCr    = 0x0b,    	

YuV   Format 
1.  Register :  0xE050  
     [1]= 1 or 0   1: Cb pixel first , 0 : Cr pixel first
     [0]= 1 or 0   1: Y pixel first ,  0 : Cb or Cr first

2. setting 
    2-1) 0xE050 <== 0x0A   Cb first   : CbY CrY CbY CrY              10
    2-2) 0xE050 <== 0x08    Cr first   : CrY  CbY CrY CbY             00
    2-3) 0xE050 <== 0x0B   Y first Cb: YCb YCr YCb YCr               11  
    2-4) 0xE050 <== 0x09    Y first Cr: YCr YCb YCr Ycb               01
==========================================================*/
    printk( "\n %s 0x%x  \n", __func__, mode);
    //CoreISP3_I2C_Read(0xE050);	//	[7:7] rw 0x00 INVVSC Invert CVS (VSYNC) output
    CoreISP3_I2C_Write(0xE050, mode);

}

void CoreISP3_PCLK_Inv(void)
{
	ClSint_16 reg_data = 0;

	reg_data = CoreISP3_I2C_Read(OutFmt);		// PCLK invert
	CoreISP3_I2C_Write(OutFmt, (reg_data | 0x20));
	printk( "reg_data 0x%x\n", reg_data);

}

void CoreISP3_TestPatten(void)
{
    printk( "\n %s \n", __func__);
/*
ISP3 Test Pattern
   Address            		Value
1. 0xE0C8                  	0x84   ( RGB value )
2. 0xE0C8                    0x85   ( Red value )
3. 0xE0C8                    0x87   ( Gray value )

*///
    //CoreISP3_I2C_Write(0xE0C8, 0x84);
    //CoreISP3_I2C_Write(0xE0C8, 0x85);
    //CoreISP3_I2C_Write(0xE0C8, 0x87);

}

void CoreISP3_Mirror_Flip(enIsp3MIRROR_MODE mode) // added by Jacky
{
#if 1  // add cmd in binary
	CoreISP3_I2C_Write(0xE628, mode);	
	CoreISP3_Send_Command(ISP3_CMD_SET_FLIP_MIRROR);
#else
	// For Samsung 4C1
	CoreISP3_I2C_Write(0xE034, 0x02);		//02h bit0	
	if(flag==0) // Normal
		CoreISP3_I2C_Write(0xE036, 0x10);
	else if(flag==1) // mirror
		CoreISP3_I2C_Write(0xE036, 0x11);
	else if(flag==2) // flip
		CoreISP3_I2C_Write(0xE036, 0x12);
	else if(flag==3) // mirror&flip
		CoreISP3_I2C_Write(0xE036, 0x13);

	
	//		0: Gr 				1: Gb 				2: Rg 				3: Bg 
	//CoreISP3_I2C_Write(0xe300, 0x02);
#endif	
}

#ifdef EEPROM_ACCESS_ENABLE

/**
 * \brief EEPROM? ??Address?? data[]? write ?? ??
 * \remarks
 * MCU? ???? ?? ???? ???? ?? ??. MCU? ??? I2C? ?? ?? ??? ??? ?? ??? ??.
 * \return
 * void
 * \author Lee Suk-Joo
 * \date 2008-06-01
 */
int ISP3_LiteOnEEPROM_Write(int startAddress, BYTE *data, int length, void (*callbackFunc)(BYTE *data, int n, int nCount))
{
 BYTE devID, addrLenType, dataLenType;
 int i=-1; // write ? ???? ?

 devID = CoreISP3_I2C_Read(0xE030);
 addrLenType = CoreISP3_I2C_Read(0xE03C);
 dataLenType = CoreISP3_I2C_Read(0xE031);

 CoreISP3_I2C_Write(0xE03C, 0x07); //# For 2Byte Length Address (it's Upper Address) - 0x05: 2Byte address, 0x07: 1Byte address
 CoreISP3_I2C_Write(0xE031, 0x51); //# IicMode1 - 0x51 : 1Byte data, 0x52 : 2Byte data

 // for(i=0; i<length; i+=2)
 for(i=0; i<length; i++)
 {
  U16 addr = startAddress+i;

  CoreISP3_I2C_Write(0xE030, DEV_ID | (addr>>8)&1);
  CoreISP3_I2C_Write(0xE034, (addr)&0xFF);
  CoreISP3_I2C_Write(0xE036, data[i]);

  if(callbackFunc != NULL)
  {
   callbackFunc(data, i, length);
  }
  else
  {
   //WaitTime_us(2*10);
   WaitTime_us(2*300);
  }
 }
 CoreISP3_I2C_Write(0xE030, devID);
 CoreISP3_I2C_Write(0xE03C, addrLenType);
 CoreISP3_I2C_Write(0xE031, dataLenType);

 return i;
}
 


/**
 * \brief EEPROM? ??Address?? data[]? read ?? ??
 * \remarks
 * MCU? ???? ?? ???? ???? ?? ??.
 * \return
 * void
 * \author Lee Suk-Joo
 * \date 2008-06-01
 */
int ISP3_LiteOnEEPROM_Read(int startAddress, BYTE *data, int length, void (*callbackFunc)(BYTE *data, int n, int nCount))
{
 BYTE devID, addrLenType, dataLenType;
 int i=-1; // read ? ???? ?

 devID = CoreISP3_I2C_Read(0xE030);
 addrLenType = CoreISP3_I2C_Read(0xE03C);
 dataLenType = CoreISP3_I2C_Read(0xE031);

 CoreISP3_I2C_Write(0xE03C, 0x07); //# For 2Byte Length Address (it's Upper Address) - 0x05: 2Byte address, 0x07: 1Byte address
 CoreISP3_I2C_Write(0xE031, 0x51); //# IicMode1 - 0x51 : 1Byte data, 0x52 : 2Byte data

 // for(i=0; i<length; i+=2)
 for(i=0; i<length; i++)
 {
  U16 addr = startAddress+i;
  CoreISP3_I2C_Write(0xE030, DEV_ID | (((addr>>8)&0x1)<<1));

  CoreISP3_I2C_Write(0xE034, addr&0xFF);
  // data[i] = ISP3_RegRead(0xE035);
  // data[i+1] = ISP3_RegRead(0xE036);
  // data[i] = ISP3_RegRead(0xE035);
  // data[i+1] = ISP3_RegRead(0xE036);
  data[i] = CoreISP3_I2C_Read(0xE036);
  data[i] = CoreISP3_I2C_Read(0xE036);
  /*
  ISP3_RegWrite(0xE034, addr&0xFF);
  // data[i] = ISP3_RegRead(0xE035);
  // data[i+1] = ISP3_RegRead(0xE036);
  // data[i] = ISP3_RegRead(0xE035);
  // data[i+1] = ISP3_RegRead(0xE036);
  data[i] = ISP3_RegRead(0xE036);
  data[i] = ISP3_RegRead(0xE036);
  */
  if(callbackFunc != NULL)
  {
   callbackFunc(data, i, length);
  }
  else
  {
   WaitTime_us(2*10);
   WaitTime_us(2*300);
  }
 }
 CoreISP3_I2C_Write(0xE030, devID);
 CoreISP3_I2C_Write(0xE03C, addrLenType);
 CoreISP3_I2C_Write(0xE031, dataLenType);

 return i;
}
 

void EEPROM_Tes_Func(void)
{
 BYTE tempdata[3300];
 int i;
 
 for(i=0;i<3300;i++)
  tempdata[i] = 0xFF;

 #if 1
 ISP3_LiteOnEEPROM_Write(0, LSC_INIT_TABLE, 0x200,  NULL);
 WaitTime_us(2*2000);
 #endif

 #if 1
 ISP3_LiteOnEEPROM_Read(0, tempdata, 0x200, Cl_Null);
 WaitTime_us(2*2000);
 for(i=0;i<0x200;i++)
 {
  if(LSC_INIT_TABLE[i] != tempdata[i])
  {
   printk( "1. ERROR data[%x] : %x :: %x\n", i, tempdata[i], LSC_INIT_TABLE[i]);
   return;
  }
  //else
  //printk( "1. OK data[%x] =%x :: LSC_INIT_TABLE[%d]=%x\n", i, data[i+1], i, LSC_INIT_TABLE[i]);
 }
 printk( ">>>>>>>>>>>>>>>>>> 1. OK\n");
 #endif

 #if 0
 ISP3_LiteOnEEPROM_Write(0x100, LSC_INIT_TABLE, 0x100,  NULL);
 WaitTime_us(2*2000);
 ISP3_LiteOnEEPROM_Read(0x100, data, 0x100, Cl_Null);
 WaitTime_us(2*2000);
 for(i=0x100;i<0xfd;i++)
 {
  if(LSC_INIT_TABLE[i] != data[i+1])
  {
   printk( "2. ERROR data[%x] : %x :: %x\n", i, data[i+1], LSC_INIT_TABLE[i]);
   return;
  }
  //else
  //printk( "2. OK data[%x] =%x :: LSC_INIT_TABLE[%d]=%x\n", i, data[i+1], i, LSC_INIT_TABLE[i]);
 }
 printk( ">>>>>>>>>>>>>>>>>> 2. OK\n");
 #endif
 
}
#endif



#ifdef ISP3_ZOOM_ENABLE
ClUint_16 ispOutputWidth;
ClUint_16 ispOutputHeight;
ClUint_16 sensorOutputWidth;
ClUint_16 sensorOutputHeight;
// Jacky add this for zoom capture
ClUint_16 ispCaptureOutputWidth;  // this is capture resolution
ClUint_16 ispCaptureOutputHeight;
ClUint_16 sensorCaptureOutputWidth = 2560;    // 5M is 2560x1920
ClUint_16 sensorCaptureOutputHeight = 1920;  // 2M is 1600x1200
//end

#ifdef DEFINE_FLOAT
float zoomStepSize = 0.1f;
float g_curDigitalZoomRate = 1.0f;
#endif


#ifndef ISP3_EVB_ENABLE
#define ROUND(d) ((d) > 0.0 ? (int) ((d) + 0.5) : -(int) ((-(d)) + 0.5))
#endif

void ISP3_RegWrite_Partial( ClUint_16 addr, ClUint_16 data2,  ClUint_16 data1, ClUint_16 data0)
{
	CoreISP3_I2C_Partial_Write(addr, data2,  data1, data0);

}

int ISP3_RegRead(ClUint_16 addr)
{
	return (int)CoreISP3_I2C_Read(addr);
}



void ISP3_Set_StillWindowsStartX(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Still_WinXStrH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_WinXStrH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Still_WinXStrL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_WinXStrL, nLow);
}

void ISP3_Set_StillWindowsStartY(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Still_WinYStrH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_WinYStrH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Still_WinYStrL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_WinYStrL, nLow);
}

void ISP3_Set_StillWindowsWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Still_WinWidthH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_WinWidthH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Still_WinWidthL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_WinWidthL, nLow);
}

void ISP3_Set_StillWindowsHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Still_WinHeightH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_WinHeightH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Still_WinHeightL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_WinHeightL, nLow);
}

void ISP3_Set_ScaleInputWindowsWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Still_SclWidthIH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_SclWidthIH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Still_SclWidthIL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_SclWidthIL, nLow);
}

void ISP3_Set_ScaleInputWindowsHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Still_SclHeightIH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_SclHeightIH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Still_SclHeightIL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_SclHeightIL, nLow);
}

void ISP3_Set_ScaleOutputWindowsWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Still_SclWidthOH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_SclWidthOH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Still_SclWidthOL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_SclWidthOL, nLow);
}

void ISP3_Set_ScaleOutputWindowsHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Still_SclHeightOH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_SclHeightOH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Still_SclHeightOL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Still_SclHeightOL, nLow);
}

void ISP3_Set_JpegWindowsWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);

	nTemp = CoreISP3_I2C_Write(JpgWidthH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(JpgWidthH, nHigh);

	nTemp = CoreISP3_I2C_Write(JpgWidthL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(JpgWidthL, nLow);
}

void ISP3_Set_JpegWindowsHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);

	nTemp = CoreISP3_I2C_Write(JpgHeightH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(JpgHeightH, nHigh);

	nTemp = CoreISP3_I2C_Write(JpgHeightL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(JpgHeightL, nLow);
}

void ISP3_Set_ThumbnailScaleInputWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Thu_SclWidthIH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Thu_SclWidthIH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Thu_SclWidthIL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Thu_SclWidthIL, nLow);
}

void ISP3_Set_ThumbnailScaleInputHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Thu_SclHeightIH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Thu_SclHeightIH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Thu_SclHeightIL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Thu_SclHeightIL, nLow);
}

void ISP3_Set_ScalePreviewWindowsInputWidth(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Pre_SclWidthIH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Pre_SclWidthIH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Pre_SclWidthIL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Pre_SclWidthIL, nLow);
}

void ISP3_Set_ScalePreviewWindowsInputHeight(ClUint_16 nData)
{
	ClUint_8 nHigh, nLow, nTemp;
	nHigh = nData>>8;
	nLow  = nData&0xFF;
	printk( "====> %s, nData: %d \n", __func__, nData);
	
	nTemp = CoreISP3_I2C_Write(Pre_SclHeightIH, nHigh);
	//if (nTemp < 0) CoreISP3_I2C_Write(Pre_SclHeightIH, nHigh);
	
	nTemp = CoreISP3_I2C_Write(Pre_SclHeightIL, nLow);
	//if (nTemp < 0) CoreISP3_I2C_Write(Pre_SclHeightIL, nLow);
}

/**
 * \brief Set Zoom for ISP3
 * \remarks
 * Change Digital Zoom rate
 * \return
 * Changed Digital Zoom rate
 * \author Lee Suk-Joo
 * \date 2008-06-01
 */
 #if 0
//float ISP3_ZoomScaleSet(int sensorOutputWidth, int sensorOutputHeight, float zoomRate, int *outputWidth, int *outputHeight)
float ISP3_ZoomScaleSet(UINT sensorOutputWidth, UINT sensorOutputHeight, float zoomRate)
{
	RETAILMSG(DBMSG, (TEXT(" CAM!++ ISP3_ZoomScaleSet.\r\n")));
	ISP3_WriteCmd(ISP3_CMD_AE_OFF);	// AE OFF

	clIIC_WriteReg_Partial_ISP3(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
//	Sleep(500);

	if(zoomRate <= ZOOMRATE_1X)
	{
		//// Downscaling ////
		//UINT width = ROUND(sensorOutputWidth * zoomRate);
		//UINT height = ROUND(sensorOutputHeight * zoomRate);
		UINT width = (UINT)((float)sensorOutputWidth * zoomRate);
		UINT height = (UINT)((float)sensorOutputHeight * zoomRate);
		RETAILMSG(DBMSG, (TEXT(" CAM!Downscaling,sensorOutputWidth = %d,sensorOutputHeight = %d.preview output width =%d,height = %d\r\n"),sensorOutputWidth,sensorOutputHeight,width,height));

		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		clIIC_WriteReg_Partial_ISP3(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
/*
		///// Sensor狼 Frame Length甫 承腮促.
*/

		ISP3_Set_StillWindowsStartX(0);
		ISP3_Set_StillWindowsStartY(0);
		ISP3_Set_StillWindowsWidth((int)sensorOutputWidth);
		ISP3_Set_StillWindowsHeight((int)sensorOutputHeight);
		ISP3_Set_ScaleInputWindowsWidth((int)sensorOutputWidth);
		ISP3_Set_ScaleInputWindowsHeight((int)sensorOutputHeight);
		ISP3_Set_ScaleOutputWindowsWidth(width);
		ISP3_Set_ScaleOutputWindowsHeight(height);
		ISP3_Set_JpegWindowsWidth(width);
		ISP3_Set_JpegWindowsHeight(height);
		ISP3_Set_ThumbnailScaleInputWidth(width);
		ISP3_Set_ThumbnailScaleInputHeight(height);
		ISP3_Set_ScalePreviewWindowsInputWidth(width);
		ISP3_Set_ScalePreviewWindowsInputHeight(height);

		//*outputWidth 	= width;
		//*outputHeight	 = height;
	}
	else
	{
		// Upscaling
		UINT startPosDiffx, startPosDiffy;

		//UINT width = ROUND(sensorOutputWidth / zoomRate);
		//UINT height = ROUND(sensorOutputHeight / zoomRate);
		UINT width = (UINT)((float)sensorOutputWidth / zoomRate);
		UINT height = (UINT)((float)sensorOutputHeight / zoomRate);

		RETAILMSG(DBMSG, (TEXT(" CAM!Downscaling,sensorOutputWidth = %d,sensorOutputHeight = %d.preview output width =%d,height = %d\r\n"),width,height,sensorOutputWidth, sensorOutputHeight));

		int valueOfE059 = CoreISP3_I2C_Read(0xE059);
		if(valueOfE059 & 0x01)
		{
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}

		clIIC_WriteReg_Partial_ISP3(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
		CoreISP3_I2C_Write(0xE012, 0x0002);
/*
		clIIC_Write(0xE033, 0x03);
		clIIC_Write(0xE034, 0x42);
		clIIC_Write(0xE035, 0x20);
		clIIC_Write(0xE036, 0x00);
*/

		if(zoomRate > 2.0f)
		{
			CoreISP3_I2C_Write(0xEDE2, 0x0004);
		}
/*
		if(g_curDigitalZoomRate == 1.0f)
		{
			clIIC_Write(0xE013, 0x0021);
			clIIC_WriteReg_Partial_ISP3(0xE014, 3, 0, 0x02);
			clIIC_Write(0xED27, 0x0001);
			clIIC_Write(0xEDE2, 0x0003);
//			clIIC_Write(0xE012, 0x0001);

			///// Sensor狼 Frame Length甫 承腮促.
		}
*/
		startPosDiffx = ((int)sensorOutputWidth - width)/2;
		startPosDiffy = ((int)sensorOutputHeight - height)/2;

		ISP3_Set_StillWindowsStartX(startPosDiffx);
		ISP3_Set_StillWindowsStartY(startPosDiffy);
		ISP3_Set_StillWindowsWidth(width);
		ISP3_Set_StillWindowsHeight(height);
		ISP3_Set_ScaleInputWindowsWidth(width);
		ISP3_Set_ScaleInputWindowsHeight(height);
		ISP3_Set_ScaleOutputWindowsWidth((int)sensorOutputWidth);
		ISP3_Set_ScaleOutputWindowsHeight((int)sensorOutputHeight);
		ISP3_Set_JpegWindowsWidth((int)sensorOutputWidth);
		ISP3_Set_JpegWindowsHeight((int)sensorOutputHeight);
		ISP3_Set_ThumbnailScaleInputWidth((int)sensorOutputWidth);
		ISP3_Set_ThumbnailScaleInputHeight((int)sensorOutputHeight);
		ISP3_Set_ScalePreviewWindowsInputWidth((int)sensorOutputWidth);
		ISP3_Set_ScalePreviewWindowsInputHeight((int)sensorOutputHeight);

		//*outputWidth 	= width;
		//*outputHeight	 = height;
	}

	clIIC_WriteReg_Partial_ISP3(0xE010, 7, 7, 1);	// Sensor 1 Input off

	ISP3_WriteCmd(ISP3_CMD_AE_ON);	// AE ON
	
	return zoomRate;
	RETAILMSG(DBMSG, (TEXT(" CAM!-- ISP3_ZoomScaleSet.\r\n")));
}

// 1280*960 (640*480) -> 1280*960
void Zoom1M_2X()
{
	ISP3_ZoomScaleSet(1280, 960, 2.0f);
}
#endif

//crop and downscale image for preview
// for 480x320  hisense
void ISP3_HVGA_PreviewZoomScaleSet(unsigned int OutputWidth, unsigned int OutputHeight, unsigned int dwZoom)
{
	unsigned int startPosDiffx, startPosDiffy,nWidth,nHeight;
	int width;
	int height;
	int valueOfE059;

	sensorOutputWidth = 1280;  // Jacky for test
	sensorOutputHeight = 960;
	ispOutputWidth = 480;   //  240   //
	ispOutputHeight = 320;  //  160  // 

      printk( "Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );
	

	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	ISP3_RegWrite_Partial(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺

	switch(dwZoom)
	{
		case CAM_ZOOM_125X:
		{
			width = 1272;    //裁剪后的大小
			height = 848;    
			break;
		}
		case CAM_ZOOM_150X:  // 1.6
		{
			width = 751;  //裁剪后的大小,640/1.6 = 400
			height = 530;	  
			break;
		}
		case CAM_ZOOM_200X:  // only this scale ok
		{
			width = 1272/2;   // 636  //裁剪后的大小
			height = 848/2;   // 424	 
			break;
		}
		case CAM_ZOOM_250X:
		{
			width = 1272;   //裁剪后的大小
			height = 848;	
			break;
		}
		case CAM_ZOOM_VP:
		{
			//width = 1056;   //裁剪后的大小,CIF*3 for 5M camera VP
			//height = 864;	
			break;
		}
		case CAM_ZOOM_100X:
		default:   //%100
		{
			width = 1272;   //
			height = 848;   //  
			break;
		}
	}		
		
	CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
	CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
	CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
	CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
	CoreISP3_I2C_Write(0xE012, 0x0001);
	
	startPosDiffx = (sensorOutputWidth - width)/2;
	startPosDiffy = (sensorOutputHeight - height)/2;

	ISP3_Set_StillWindowsStartX(startPosDiffx);
	ISP3_Set_StillWindowsStartY(startPosDiffy);
	ISP3_Set_StillWindowsWidth(width);
	ISP3_Set_StillWindowsHeight(height);
	ISP3_Set_ScaleInputWindowsWidth(width);
	ISP3_Set_ScaleInputWindowsHeight(height);
	
	ISP3_Set_ScaleOutputWindowsWidth(ispOutputWidth); 
	ISP3_Set_ScaleOutputWindowsHeight(ispOutputHeight);
	ISP3_Set_JpegWindowsWidth(ispOutputWidth);
	ISP3_Set_JpegWindowsHeight(ispOutputHeight);
	ISP3_Set_ThumbnailScaleInputWidth(ispOutputWidth);
	ISP3_Set_ThumbnailScaleInputHeight(ispOutputHeight);
	ISP3_Set_ScalePreviewWindowsInputWidth(ispOutputWidth);
	ISP3_Set_ScalePreviewWindowsInputHeight(ispOutputHeight);


	ISP3_RegWrite_Partial(0xE010, 7, 7, 1);	// Sensor 1 Input off

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
    	printk( " %s %dx%d, zoom: %d \n",__func__, OutputWidth, OutputHeight, dwZoom);
		
	
}

void ISP3_VGA_PreviewZoomScaleSet(unsigned int OutputWidth, unsigned int OutputHeight, unsigned int dwZoom)
{
	unsigned int startPosDiffx, startPosDiffy,nWidth,nHeight;
	
    	printk( " %s %dx%d, zoom: %d \n",__func__, OutputWidth, OutputHeight, dwZoom);
		
	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
	//Sleep(500);
	switch(dwZoom)
	{
		case CAM_ZOOM_100X:
		{
			nWidth = 640;
			nHeight = 480;		
			break;
		}
		case CAM_ZOOM_125X:
		{
			nWidth = 512;   //裁剪后的大小
			nHeight = 384;
			break;
		}
		case CAM_ZOOM_150X:  // 1.6
		{
			nWidth = 400;  //裁剪后的大小,640/1.6 = 400
			nHeight = 300;	  
			break;
		}
		case CAM_ZOOM_200X:  // only this scale ok
		{
			nWidth = 320;   //裁剪后的大小
			nHeight = 240;	
			break;
		}
		case CAM_ZOOM_250X:
		{
			nWidth = 256;   //裁剪后的大小
			nHeight = 192;	
			break;
		}
		case CAM_ZOOM_VP:
		{
			//nWidth = 1056;   //裁剪后的大小,CIF*3 for 5M camera VP
			//nHeight = 864;	
			break;
		}
		default:   //%100
		{
			nWidth = 640;
			nHeight = 480;		
			break;
		}
	}

	startPosDiffx = (640 - nWidth)/2;
	startPosDiffy = (480 - nHeight)/2;	

	CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
	CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
	CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
	CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
	CoreISP3_I2C_Write(0xE012, 0x0001);

	ISP3_Set_StillWindowsStartX(startPosDiffx);
	ISP3_Set_StillWindowsStartY(startPosDiffy);
	ISP3_Set_StillWindowsWidth(nWidth);  // width
	ISP3_Set_StillWindowsHeight(nHeight);
	ISP3_Set_ScaleInputWindowsWidth(nWidth);
	ISP3_Set_ScaleInputWindowsHeight(nHeight);
	ISP3_Set_ScaleOutputWindowsWidth(OutputWidth);
	ISP3_Set_ScaleOutputWindowsHeight(OutputHeight);	
	
	ISP3_Set_JpegWindowsWidth(OutputWidth);
	ISP3_Set_JpegWindowsHeight(OutputHeight);
	ISP3_Set_ThumbnailScaleInputWidth(OutputWidth);
	ISP3_Set_ThumbnailScaleInputHeight(OutputHeight);
	ISP3_Set_ScalePreviewWindowsInputWidth(OutputWidth);
	ISP3_Set_ScalePreviewWindowsInputHeight(OutputHeight);

	//Sleep(500);
	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 1);	// Sensor 1 Input on

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
	
}


//crop and downscale image for preview
void ISP3_PreviewZoomScaleSet(unsigned int OutputWidth, unsigned int OutputHeight, unsigned int dwZoom)
{
	unsigned int startPosDiffx, startPosDiffy,nWidth,nHeight;
	
    	printk( " %s %dx%d, zoom: %d \n",__func__, OutputWidth, OutputHeight, dwZoom);
		
	//CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
	//Sleep(500);
	switch(dwZoom)
	{
		case CAM_ZOOM_100X:
		{
			nWidth = 1280;
			nHeight = 960;		
			break;
		}
		case CAM_ZOOM_125X:
		{
			nWidth = 1024;   //裁剪后的大小
			nHeight = 768;
			break;
		}
		case CAM_ZOOM_150X:
		{
			nWidth = 864;  //裁剪后的大小,1280/864 = 1.48
			nHeight = 648;	  //960/648 = 1.48
			break;
		}
		case CAM_ZOOM_200X:  // 2X
		{
			nWidth = 640;   //裁剪后的大小
			nHeight = 480;	
			break;
		}
		case CAM_ZOOM_250X:
		{
			nWidth = 512;   //裁剪后的大小
			nHeight = 384;	
			break;
		}
		case CAM_ZOOM_VP:
		{
			nWidth = 1056;   //裁剪后的大小,CIF*3 for 5M camera VP
			nHeight = 864;	
			break;
		}
		default:   //%100
		{
			nWidth = 1280;
			nHeight = 960;		
			break;
		}
	}

	startPosDiffx = (1280 - nWidth)/2;
	startPosDiffy = (960 - nHeight)/2;	

	/*  can not disable tollowing code */
	CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
	CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
	CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
	CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
	CoreISP3_I2C_Write(0xE012, 0x0001);
	
	
	ISP3_Set_StillWindowsStartX(startPosDiffx);
	ISP3_Set_StillWindowsStartY(startPosDiffy);
	ISP3_Set_StillWindowsWidth(nWidth);
	ISP3_Set_StillWindowsHeight(nHeight);
	ISP3_Set_ScaleInputWindowsWidth(nWidth);
	ISP3_Set_ScaleInputWindowsHeight(nHeight);
	ISP3_Set_ScaleOutputWindowsWidth(OutputWidth);
	ISP3_Set_ScaleOutputWindowsHeight(OutputHeight);
/*	
	ISP3_Set_JpegWindowsWidth(OutputWidth);
	ISP3_Set_JpegWindowsHeight(OutputHeight);
	ISP3_Set_ThumbnailScaleInputWidth(OutputWidth);
	ISP3_Set_ThumbnailScaleInputHeight(OutputHeight);
	ISP3_Set_ScalePreviewWindowsInputWidth(OutputWidth);
	ISP3_Set_ScalePreviewWindowsInputHeight(OutputHeight);
*/
	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 1);	// Sensor 1 Input on

	//CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
	
}

//crop and downscale image for preview
void ISP3_CaptureZoomScaleSet(unsigned int OutputWidth, unsigned int OutputHeight, unsigned int dwZoom)
{
	unsigned int startPosDiffx, startPosDiffy,nWidth,nHeight;
	
	sensorOutputWidth = 2560;  //
	sensorOutputHeight = 1920;
	
	ispCaptureOutputWidth = 2560;
	ispCaptureOutputWidth = 1920;
	
	ispOutputWidth = 2560;   //  240   //
	ispOutputHeight = 1920;  //  160  // 
	
    	printk( " %s %dx%d, zoom: %d \n",__func__, OutputWidth, OutputHeight, dwZoom);
		
	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
	//Sleep(500);
	switch(dwZoom)
	{
		case CAM_ZOOM_100X:
		{
			nWidth = sensorOutputWidth;
			nHeight = sensorOutputHeight;		
			break;
		}
		case CAM_ZOOM_125X:
		{
			nWidth = sensorOutputWidth*4/5;   //裁剪后的大小
			nHeight = sensorOutputHeight*4/5;
			break;
		}
		case CAM_ZOOM_150X:  // 1.6
		{
			nWidth = sensorOutputWidth*4/5;   //裁剪后的大小
			nHeight = sensorOutputHeight*4/5;
			break;
		}
		case CAM_ZOOM_200X:
		{
			nWidth = sensorOutputWidth/2;   //裁剪后的大小
			nHeight = sensorOutputHeight/2;
			break;
		}
		case CAM_ZOOM_250X:
		{
			nWidth = sensorOutputWidth*2/5;   //裁剪后的大小
			nHeight = sensorOutputWidth*2/5;	
			break;
		}
		case CAM_ZOOM_VP:
		{
			nWidth = 1056;   //裁剪后的大小,CIF*3 for 5M camera VP
			nHeight = 864;	
			break;
		}
		default:   //%100
		{
			nWidth = 1280;
			nHeight = 960;		
			break;
		}
	}

	startPosDiffx = (2560 - nWidth)/2;
	startPosDiffy = (1920 - nHeight)/2;	

	CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
	CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
	CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
	CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
	CoreISP3_I2C_Write(0xE012, 0x0001);

	ISP3_Set_StillWindowsStartX(startPosDiffx);
	ISP3_Set_StillWindowsStartY(startPosDiffy);
	ISP3_Set_StillWindowsWidth(nWidth);
	ISP3_Set_StillWindowsHeight(nHeight);
	ISP3_Set_ScaleInputWindowsWidth(nWidth);
	ISP3_Set_ScaleInputWindowsHeight(nHeight);
	
	ISP3_Set_ScaleOutputWindowsWidth(OutputWidth);
	ISP3_Set_ScaleOutputWindowsHeight(OutputHeight);
	ISP3_Set_JpegWindowsWidth(OutputWidth);
	ISP3_Set_JpegWindowsHeight(OutputHeight);
	ISP3_Set_ThumbnailScaleInputWidth(OutputWidth);
	ISP3_Set_ThumbnailScaleInputHeight(OutputHeight);
	ISP3_Set_ScalePreviewWindowsInputWidth(OutputWidth);
	ISP3_Set_ScalePreviewWindowsInputHeight(OutputHeight);

	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 1);	// Sensor 1 Input on

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
}

void ISP3_PreviewZoomScaleSet_YuLong(unsigned int OutputWidth, unsigned int OutputHeight, unsigned int dwZoom)
{
	unsigned int startPosDiffx, startPosDiffy,nWidth,nHeight;
	
	
	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
	//Sleep(500);
	switch(dwZoom)
	{
		case CAM_ZOOM_100X:
		{
			nWidth = 1280;
			nHeight = 960;		
			break;
		}
		case CAM_ZOOM_125X:
		{
			nWidth = 1024;   //裁剪后的大小
			nHeight = 768;
			break;
		}
		case CAM_ZOOM_150X:
		{
			nWidth = 864;  //裁剪后的大小,1280/864 = 1.48
			nHeight = 648;	  //960/648 = 1.48
			break;
		}
		case CAM_ZOOM_200X:
		{
			nWidth = 640;   //裁剪后的大小
			nHeight = 480;	
			break;
		}
		case CAM_ZOOM_250X:
		{
			nWidth = 512;   //裁剪后的大小
			nHeight = 384;	
			break;
		}
		case CAM_ZOOM_VP:
		{
			nWidth = 1056;   //裁剪后的大小,CIF*3 for 5M camera VP
			nHeight = 864;	
			break;
		}
		default:   //%100
		{
			nWidth = 1280;
			nHeight = 960;		
			break;
		}
	}

	startPosDiffx = (1280 - nWidth)/2;
	startPosDiffy = (960 - nHeight)/2;	

	CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
	CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
	CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
	CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
	CoreISP3_I2C_Write(0xE012, 0x0001);
/*	if(nWidth >= OutputWidth)  //downscale
	{
		RETAILMSG(DEBUG_INFOR, (TEXT(" CAM!downscale.\r\n")));
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
	}
	else   //upscale
	{
		RETAILMSG(DEBUG_INFOR, (TEXT(" CAM!upscale.\r\n")));
		int valueOfE059 = CoreISP3_I2C_Read(0xE059);
		if(valueOfE059 & 0x01)
		{
			RETAILMSG(DEBUG_INFOR, (TEXT(" CAM!CoreISP3_I2C_Write(0xE013, 0x0021);.\r\n")));
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			RETAILMSG(DEBUG_INFOR, (TEXT(" CAM!CoreISP3_I2C_Write(0xE013, 0x0022);.\r\n")));
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}

		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
		CoreISP3_I2C_Write(0xE012, 0x0002);
	}
*/	
	ISP3_Set_StillWindowsStartX(startPosDiffx);
	ISP3_Set_StillWindowsStartY(startPosDiffy);
	ISP3_Set_StillWindowsWidth(nWidth);
	ISP3_Set_StillWindowsHeight(nHeight);
	ISP3_Set_ScaleInputWindowsWidth(nWidth);
	ISP3_Set_ScaleInputWindowsHeight(nHeight);
	ISP3_Set_ScaleOutputWindowsWidth(OutputWidth);
	ISP3_Set_ScaleOutputWindowsHeight(OutputHeight);
	ISP3_Set_JpegWindowsWidth(OutputWidth);
	ISP3_Set_JpegWindowsHeight(OutputHeight);
	ISP3_Set_ThumbnailScaleInputWidth(OutputWidth);
	ISP3_Set_ThumbnailScaleInputHeight(OutputHeight);
	ISP3_Set_ScalePreviewWindowsInputWidth(OutputWidth);
	ISP3_Set_ScalePreviewWindowsInputHeight(OutputHeight);

	//Sleep(500);
	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 1);	// Sensor 1 Input on

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
	
}
//crop and downscale image for capture
void ISP3_ImageCrop(unsigned int sensorOutputWidth, unsigned int sensorOutputHeight,unsigned int CropedWidth,unsigned int CropedHeight, unsigned int OutputWidth,unsigned int OutputHeight)
{
	unsigned int startPosDiffx, startPosDiffy;
	int valueOfE059;
	
	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 0);	// Sensor 1 Input off 
	WaitTime_us(2*50*1000);

	startPosDiffx = (sensorOutputWidth - CropedWidth)/2;
	startPosDiffy = (sensorOutputHeight - CropedHeight)/2;	

    	printk( " %s %dx%d \n",__func__, startPosDiffx, startPosDiffy);

	#if 1  // for JPEG zoom capture must use following code...test ok
	valueOfE059 = ISP3_RegRead(0xE059);
	
	printk( " 0xe059: 0x%x \n", valueOfE059);
	if(valueOfE059 & 0x01)
	{
		// JPEG 捞搁
		CoreISP3_I2C_Write(0xE013, 0x0021);
	}
	else
	{
		// YCbCr 捞搁
		CoreISP3_I2C_Write(0xE013, 0x0022);
	}
	ISP3_RegWrite_Partial(0xE014, 3, 0, 0x01);
	CoreISP3_I2C_Write(0xED27, 0x0011);
	CoreISP3_I2C_Write(0xEDE2, 0x0003);
		CoreISP3_I2C_Write(0xE012, 0x0002);

	//if(zoomRate > 2.0f)
	{
		//CoreISP3_I2C_Write(0xEDE2, 0x0004);
	}
	#else// jacky open this for test ok
	CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
	CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
	CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
	CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
	CoreISP3_I2C_Write(0xE012, 0x0001);
	#endif
	
	ISP3_Set_StillWindowsStartX(startPosDiffx);
	ISP3_Set_StillWindowsStartY(startPosDiffy);
	ISP3_Set_StillWindowsWidth(CropedWidth);
	ISP3_Set_StillWindowsHeight(CropedHeight);
	ISP3_Set_ScaleInputWindowsWidth(CropedWidth);
	ISP3_Set_ScaleInputWindowsHeight(CropedHeight);
	ISP3_Set_ScaleOutputWindowsWidth(OutputWidth);
	ISP3_Set_ScaleOutputWindowsHeight(OutputHeight);
	// following code  need for jpeg 
	ISP3_Set_JpegWindowsWidth(OutputWidth);
	ISP3_Set_JpegWindowsHeight(OutputHeight);
	ISP3_Set_ThumbnailScaleInputWidth(OutputWidth);
	ISP3_Set_ThumbnailScaleInputHeight(OutputHeight);
	ISP3_Set_ScalePreviewWindowsInputWidth(OutputWidth);
	ISP3_Set_ScalePreviewWindowsInputHeight(OutputHeight);
/**/
	WaitTime_us(2*50);
	CoreISP3_I2C_Partial_Write(0xE010, 7, 7, 1);	// Sensor 1 Input on

}

void ISP3_ZoomInit(void)
{
	int upscaleStepCount;
	int downscaleStepCount;

	#ifdef ISP3_EVB_ENABLE
	printk( "====> %s, [%dx%d]\n", __func__, g_Isp3SensorInfo.width, g_Isp3SensorInfo.height);

       sensorOutputWidth = CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL);
      	sensorOutputHeight = CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL);
      	ispOutputWidth = CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL);
      	ispOutputHeight= CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL);
	#else
       sensorOutputWidth = CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL);
      	sensorOutputHeight = CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL);
      	ispOutputWidth = CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL);
      	ispOutputHeight= CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL);
	#endif
	printk( "====> %s,   I[%d %d]->O[%d %d]\n", __func__, sensorOutputWidth, sensorOutputHeight, ispOutputWidth, ispOutputHeight);

	#ifdef DEFINE_FLOAT
	zoomStepSize = 0.1f;
	// SlideBar 狼 Range绰 	0 ~ upscaleStepCount + downscaleStepCount 捞 等促.
	g_curDigitalZoomRate = 1.0f;
	#endif
	
}

void Yulong_TestZoom(void)
{
#ifdef DEFINE_FLOAT
	printk( "====> %s \n", __func__);
	/////////////////////////////////////
	// Zoom In Full mode
	/////////////////////////////////////
	CoreISP3_SetResolution(enISP_RES_5MP_FULL , CMD_Preview);

	sensorOutputWidth = 2560;
	sensorOutputHeight = 1920;
	ispOutputWidth = 2560;
	ispOutputHeight = 1920;
/////// up scaleing /////////
	ISP3_SetDigitalZoomRate((float)2560/(float)1280);	// 2.0
	ISP3_SetDigitalZoomRate((float)2560/(float)1600);	// 1.6
	ISP3_SetDigitalZoomRate((float)2560/(float)2048);	// 1.25
	ISP3_SetDigitalZoomRate((float)2560/(float)2560);	// 1.0

/////// down scaleing /////////
	ISP3_SetDigitalZoomRate((float)2048/(float)2560);
	ISP3_SetDigitalZoomRate((float)1600/(float)2560);
	ISP3_SetDigitalZoomRate((float)1280/(float)2560);
	ISP3_SetDigitalZoomRate((float)1024/(float)2560);
	ISP3_SetDigitalZoomRate((float)800/(float)2560);
	ISP3_SetDigitalZoomRate((float)640/(float)2560);
	ISP3_SetDigitalZoomRate((float)320/(float)2560);
	ISP3_SetDigitalZoomRate((float)160/(float)2560);

	/////////////////////////////////////
	// Zoom In Preview mode
	/////////////////////////////////////
	//CoreISP3_SetResolution(ISP3_CMD_PREVIEW, CMD_Preview);
	CoreISP3_SetResolution(enISP_RES_5MP_FULL, CMD_Preview);  // Jacky change this
	
	sensorOutputWidth = 1280;
	sensorOutputHeight = 960;
	ispOutputWidth = 1280;
	ispOutputHeight = 960;
	
/////// up scaleing /////////
	ISP3_SetDigitalZoomRate((float)1280/(float)640);	// 2.0
	ISP3_SetDigitalZoomRate((float)1280/(float)800);
	ISP3_SetDigitalZoomRate((float)1280/(float)1024);
	ISP3_SetDigitalZoomRate((float)1280/(float)1280);	// 1.0

/////// down scaleing /////////
	ISP3_SetDigitalZoomRate((float)1024/(float)1280);
	ISP3_SetDigitalZoomRate((float)800/(float)1280);
	ISP3_SetDigitalZoomRate((float)640/(float)1280);
	ISP3_SetDigitalZoomRate((float)320/(float)1280);
	ISP3_SetDigitalZoomRate((float)160/(float)1280);

	/////////////////////////////////////
	// Zoom In VGA mode
	/////////////////////////////////////
	CoreISP3_SetResolution(enISP_RES_VGA, CMD_Preview);  // Jacky change this
	
	sensorOutputWidth = 640;
	sensorOutputHeight = 480;
	ispOutputWidth = 640;
	ispOutputHeight = 480;

/////// up scaleing /////////
	ISP3_SetDigitalZoomRate((float)640/(float)320);		// 2.0
	ISP3_SetDigitalZoomRate((float)640/(float)640);		// 1.0

/////// down scaleing /////////
	ISP3_SetDigitalZoomRate((float)320/(float)640);		// 0.5
	ISP3_SetDigitalZoomRate((float)160/(float)640);		// 0.25
#endif	
}

/*

1280x960 (1272*848)  -> 480x320
//(1280-1272)/2 = 4
//(960-848)/2 = 56

OPPO request
  高宽比例为5：3   //可以实现，Corelogic方争取尽早完成
   800*480
   1600*960
   2048*1228
   2560*1536

*/

void ISP3_SetDigitalZoom(CAM_ZOOM_T rate)  // for 480x320 resolution
{
	int width;
	int height;
	int startPosDiffx, startPosDiffy;
	int valueOfE059;

	//Image scaling: 1/32 ~ 2x linear (area 4x)

	sensorOutputWidth = 1280;  // Jacky for test
	sensorOutputHeight = 960;
	ispOutputWidth = 480;   //  240   //
	ispOutputHeight = 320;  //  160  // 

      printk( "ISP3_SetDigitalZoom    Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
					      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
					      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
					      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );
	

	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	ISP3_RegWrite_Partial(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺

	{
		// Upscaling
		switch(rate)
		{
			case CAM_ZOOM_125X:
			{
				width = 1272-120;  // 1152        //裁剪后的大小
				height = 848-80;   //  768  
				break;
			}
			case CAM_ZOOM_150X:  // 1.6
			{
				width = 751;  //裁剪后的大小,640/1.6 = 400
				height = 530;	  
				break;
			}
			case CAM_ZOOM_200X:  // only this scale ok
			{
				width = 1272/2;   // 636  //裁剪后的大小
				height = 848/2;   // 424	 
				break;
			}
			case CAM_ZOOM_250X:
			{
				width = 636-30;  // 606  //裁剪后的大小
				height = 424-20;  // 404	
				break;
			}
			case CAM_ZOOM_400X:
			{
				width = 480;   
				height = 320;	
				break;
			}
			//		Image scaling: 1/32 ~ 2x linear (area 4x) 	
			case CAM_ZOOM_CIF:
			{
				ispOutputWidth = 352;   //  240   //   176x144 ->88x72 ->44x36->22x18->11x9   ->110x90
				ispOutputHeight = 288;  //  160  // 
				width = 1166;   //             1408x1152    - 176x144  = 1232x1008           -176x144 = 1056x864
				height = 954;   //    960
				break;
			}
				
			case CAM_ZOOM_QCIF:
			{ 
				ispOutputWidth = 176;   //  240   //   176x144 ->88x72 ->44x36->22x18->11x9   ->110x90
				ispOutputHeight = 144;  //  160  // 
				width = 1166;   //             1408x1152    - 176x144  = 1232x1008           -176x144 = 1056x864
				height = 954;   //    960
				break;
			}
			case CAM_ZOOM_480_360:   // isp scale to 480x360
			{ 
				ispOutputWidth = 480;   
				ispOutputHeight = 360; 
				width = 1280;   
				height = 960;  
				break;
			}
			case CAM_ZOOM_480_360X1:   // isp scale to 480x360
			{ 
				ispOutputWidth = 480;   
				ispOutputHeight = 360; 
				width = 1120;   
				height = 840;  
				break;
			}
			case CAM_ZOOM_480_360X2:   // isp scale to 480x360
			{ 
				ispOutputWidth = 480;   
				ispOutputHeight = 360; 
				width = 960;	//640;   
				height = 720;	//480;  
				break;
			}
			case CAM_ZOOM_480_360X3:   // isp scale to 480x360
			{ 
				ispOutputWidth = 480;   
				ispOutputHeight = 360; 
				width = 800;	//520;   
				height = 600;	//390;  
				break;
			}
			case CAM_ZOOM_480_360X4:   // isp scale to 480x360
			{ 
				ispOutputWidth = 480;   
				ispOutputHeight = 360; 
				width = 640;	//480;   
				height = 480;	//360;  
				break;
			}
			case CAM_ZOOM_VGA:   // isp scale to 
			{ 
				ispOutputWidth = 640;   
				ispOutputHeight = 480; 
				width = 1280;   
				height = 960;  
				break;
			}
			case CAM_ZOOM_SVGA:   // isp scale to 
			{ 
				ispOutputWidth = 800;   
				ispOutputHeight = 600; 
				width = 1280;   
				height = 960;  
				break;
			}
			case CAM_ZOOM_1_3M:   // isp scale to 
			{ 
				ispOutputWidth = 1280;   
				ispOutputHeight = 960; 
				width = 1280;   
				height = 960;  
				break;
			}

			// For OPPO  5:3 resolutions
			case CAM_ZOOM_800_480:   // isp scale to 
			{    // 1280x960 crop and scale down
				ispOutputWidth = 800;   
				ispOutputHeight = 480; 
				width = 1280;     
				height = 768;    // 960-768=192
				break;
			}
			case CAM_ZOOM_1600_960:   // isp scale to 
			{    // 2560x1920 crop and scale down
				sensorOutputWidth = 2560;  // Jacky for test
				sensorOutputHeight = 1920;
				ispOutputWidth = 1600;   
				ispOutputHeight = 960; 
				width = 2560;     
				height = 1536;    //1920-1536 = 384
				break;
			}
			case CAM_ZOOM_2048_1228:   // isp scale to 
			{    // 2560x1920 crop and scale down
				sensorOutputWidth = 2560;  // Jacky for test
				sensorOutputHeight = 1920;
				ispOutputWidth = 2048;   
				ispOutputHeight = 1228;   //2048*0.6  =  1228.8
				width = 2560;     
				height = 1535;         // 2560x1535->2048x1228              1920-1535=385
				break;
			}
			case CAM_ZOOM_2560_1536:   // isp scale to 
			{    // 2560x1920 crop and scale down
				sensorOutputWidth = 2560;  // Jacky for test
				sensorOutputHeight = 1920;
				ispOutputWidth = 2560;   
				ispOutputHeight = 1536; 
				width = 2560;     
				height = 1536;    //1920-1536 = 384
				break;
			}
			
			case CAM_ZOOM_100X:
			default:   //%100
			{
				width = 1272;   //
				height = 848;   //  
				break;
			}
		}			

		#if 1
		valueOfE059 = ISP3_RegRead(0xE059);
		//printk( "Upscaling  [%d %d], 0xe059: 0x%x \n", width, height, valueOfE059);
		if(valueOfE059 & 0x01)
		{
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}
		ISP3_RegWrite_Partial(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
 		CoreISP3_I2C_Write(0xE012, 0x0002);
		#else		
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
		#endif
		
		startPosDiffx = (sensorOutputWidth - width)/2;
		startPosDiffy = (sensorOutputHeight - height)/2;

		ISP3_Set_StillWindowsStartX(startPosDiffx);
		ISP3_Set_StillWindowsStartY(startPosDiffy);
		ISP3_Set_StillWindowsWidth(width);
		ISP3_Set_StillWindowsHeight(height);
		ISP3_Set_ScaleInputWindowsWidth(width);
		ISP3_Set_ScaleInputWindowsHeight(height);
		ISP3_Set_ScaleOutputWindowsWidth(ispOutputWidth); // Jacky change and test ok
		ISP3_Set_ScaleOutputWindowsHeight(ispOutputHeight);
		
		/*// following ..   do not set following register, it will effect face tracking
		ISP3_Set_JpegWindowsWidth(ispOutputWidth);
		ISP3_Set_JpegWindowsHeight(ispOutputHeight);
		ISP3_Set_ThumbnailScaleInputWidth(ispOutputWidth);
		ISP3_Set_ThumbnailScaleInputHeight(ispOutputHeight);
		ISP3_Set_ScalePreviewWindowsInputWidth(ispOutputWidth);
		ISP3_Set_ScalePreviewWindowsInputHeight(ispOutputHeight);
		*/
	}

	ISP3_RegWrite_Partial(0xE010, 7, 7, 1);	// Sensor 1 Input off

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON   Jacky disable this for test
	
}


#ifdef DEFINE_FLOAT
float ISP3_SetDigitalZoomRate(float zoomRate)
{
	int width;
	int height;

	#if 0
       sensorOutputWidth = CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL);
      	sensorOutputHeight = CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL);
      	ispOutputWidth = CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL);
      	ispOutputHeight= CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL);
	//#else
	sensorOutputWidth = 1280;  // Jacky for test
	sensorOutputHeight = 960;
	ispOutputWidth = 640;
	ispOutputHeight = 480;
	#endif
      printk( "Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );
	
	printk( "====> %s, rate: %d    I[%d %d]->O[%d %d]\n", __func__, zoomRate, sensorOutputWidth, sensorOutputHeight, ispOutputWidth, ispOutputHeight);

	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	ISP3_RegWrite_Partial(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺

	if(zoomRate <= ZOOMRATE_1X)
	{
		//// Downscaling ////

		width = ROUND((float)sensorOutputWidth * zoomRate);
		height = ROUND((float)sensorOutputHeight * zoomRate);
		
		printk( "Downscaling  [%d %d] \n", width, height);

		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		ISP3_RegWrite_Partial(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
 		CoreISP3_I2C_Write(0xE012, 0x0001);

		ISP3_Set_StillWindowsStartX(0);
		ISP3_Set_StillWindowsStartY(0);
		ISP3_Set_StillWindowsWidth(sensorOutputWidth);
		ISP3_Set_StillWindowsHeight(sensorOutputHeight);
		ISP3_Set_ScaleInputWindowsWidth(sensorOutputWidth);
		ISP3_Set_ScaleInputWindowsHeight(sensorOutputHeight);
		ISP3_Set_ScaleOutputWindowsWidth(width);
		ISP3_Set_ScaleOutputWindowsHeight(height);
		/*
		ISP3_Set_JpegWindowsWidth(width);
		ISP3_Set_JpegWindowsHeight(height);
		ISP3_Set_ThumbnailScaleInputWidth(width);
		ISP3_Set_ThumbnailScaleInputHeight(height);
		ISP3_Set_ScalePreviewWindowsInputWidth(width);
		ISP3_Set_ScalePreviewWindowsInputHeight(height);
		*/

	}
	else
	{
		// Upscaling
		int startPosDiffx, startPosDiffy;
		int valueOfE059;
			
		//width = ROUND((float)sensorOutputWidth * zoomRate);  // Jacky for test
		//height = ROUND((float)sensorOutputHeight * zoomRate);
		width = ((float)sensorOutputWidth / zoomRate);   // ROUND
		height = ((float)sensorOutputHeight / zoomRate);  //ROUND
		
		valueOfE059 = ISP3_RegRead(0xE059);
		
		printk( "Upscaling  [%d %d], 0xe059: 0x%x \n", width, height, valueOfE059);
		#if 1  // test ok
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
		#else// jacky open this for test ok
		if(valueOfE059 & 0x01)
		{
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}
		ISP3_RegWrite_Partial(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
 		CoreISP3_I2C_Write(0xE012, 0x0002);

		if(zoomRate > 2.0f)
		{
			CoreISP3_I2C_Write(0xEDE2, 0x0004);
		}
		#endif
		
		startPosDiffx = (sensorOutputWidth - width)/2;
		startPosDiffy = (sensorOutputHeight - height)/2;

		ISP3_Set_StillWindowsStartX(startPosDiffx);
		ISP3_Set_StillWindowsStartY(startPosDiffy);
		ISP3_Set_StillWindowsWidth(width);
		ISP3_Set_StillWindowsHeight(height);
		ISP3_Set_ScaleInputWindowsWidth(width);
		ISP3_Set_ScaleInputWindowsHeight(height);
		ISP3_Set_ScaleOutputWindowsWidth(ispOutputWidth); // Jacky change and test ok
		ISP3_Set_ScaleOutputWindowsHeight(ispOutputHeight);
		/*
		ISP3_Set_JpegWindowsWidth(ispOutputWidth);
		ISP3_Set_JpegWindowsHeight(ispOutputHeight);
		ISP3_Set_ThumbnailScaleInputWidth(ispOutputWidth);
		ISP3_Set_ThumbnailScaleInputHeight(ispOutputHeight);
		ISP3_Set_ScalePreviewWindowsInputWidth(ispOutputWidth);
		ISP3_Set_ScalePreviewWindowsInputHeight(ispOutputHeight);
		*/
	}

	ISP3_RegWrite_Partial(0xE010, 7, 7, 1);	// Sensor 1 Input off

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
	return zoomRate;
}


float ISP3_SetDigitalZoomRate_Face(float zoomRate, int startPosDiffx, int startPosDiffy)
{
	int width;
	int height;

	#if 0
	sensorOutputWidth = 1280;  // Jacky for test
	sensorOutputHeight = 960;
	ispOutputWidth = 640;
	ispOutputHeight = 480;
	#endif
      printk( "Scl Window[%d  %d  %d %d] \n", CoreISP3_I2C_Read(Still_SclWidthIH)<<8|CoreISP3_I2C_Read(Still_SclWidthIL)
      													      , CoreISP3_I2C_Read(Still_SclHeightIH)<<8|CoreISP3_I2C_Read(Still_SclHeightIL)
      													      , CoreISP3_I2C_Read(Still_SclWidthOH)<<8|CoreISP3_I2C_Read(Still_SclWidthOL)
      													      , CoreISP3_I2C_Read(Still_SclHeightOH)<<8|CoreISP3_I2C_Read(Still_SclHeightOL)
      													      );
	
	printk( "====> %s, rate%d:  I[%d %d]->O[%d %d]\n", __func__, sensorOutputWidth, sensorOutputHeight, ispOutputWidth, ispOutputHeight);

	CoreISP3_Send_Command(ISP3_CMD_AE_OFF);	// AE OFF

	ISP3_RegWrite_Partial(0xE010, 7, 7, 0);	// Sensor 1 Input off - IPC 贸府 档吝 Scale 悸泼捞 函版登搁 巩力啊 惯积登骨肺
//	Sleep(500);

	if(zoomRate <= ZOOMRATE_1X)
	{
		//// Downscaling ////

		width = ROUND((float)sensorOutputWidth * zoomRate);
		height = ROUND((float)sensorOutputHeight * zoomRate);
		
		printk( "Downscaling  [%d %d] \n", width, height);

		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		ISP3_RegWrite_Partial(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
 		CoreISP3_I2C_Write(0xE012, 0x0001);

		ISP3_Set_StillWindowsStartX(0);
		ISP3_Set_StillWindowsStartY(0);
		ISP3_Set_StillWindowsWidth(sensorOutputWidth);
		ISP3_Set_StillWindowsHeight(sensorOutputHeight);
		ISP3_Set_ScaleInputWindowsWidth(sensorOutputWidth);
		ISP3_Set_ScaleInputWindowsHeight(sensorOutputHeight);
		ISP3_Set_ScaleOutputWindowsWidth(width);
		ISP3_Set_ScaleOutputWindowsHeight(height);
		/*
		ISP3_Set_JpegWindowsWidth(width);
		ISP3_Set_JpegWindowsHeight(height);
		ISP3_Set_ThumbnailScaleInputWidth(width);
		ISP3_Set_ThumbnailScaleInputHeight(height);
		ISP3_Set_ScalePreviewWindowsInputWidth(width);
		ISP3_Set_ScalePreviewWindowsInputHeight(height);
		*/

	}
	else
	{
		// Upscaling
		//int startPosDiffx, startPosDiffy;
		int valueOfE059;
			
		//width = ROUND((float)sensorOutputWidth * zoomRate);  // Jacky for test
		//height = ROUND((float)sensorOutputHeight * zoomRate);
		width = ((float)sensorOutputWidth / zoomRate);   // ROUND
		height = ((float)sensorOutputHeight / zoomRate);  //ROUND
		
		#if 1  // test ok
		CoreISP3_I2C_Write(0xE013, 0x0021);	// Cis1IntD
		CoreISP3_I2C_Partial_Write(0xE014, 3, 0, 0x02);	// Cis1IntE
		CoreISP3_I2C_Write(0xED27, 0x0001);	// Still_ClkMode_Bypss
		CoreISP3_I2C_Write(0xEDE2, 0x0003);	// Mem_Type
		CoreISP3_I2C_Write(0xE012, 0x0001);
		#else// jacky open this for test
		valueOfE059 = ISP3_RegRead(0xE059);
		printk( "Upscaling  [%d %d], 0xe059: 0x%x \n", width, height, valueOfE059);
		
		if(valueOfE059 & 0x01)
		{
			// JPEG 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0021);
		}
		else
		{
			// YCbCr 捞搁
			CoreISP3_I2C_Write(0xE013, 0x0022);
		}
		ISP3_RegWrite_Partial(0xE014, 3, 0, 0x01);
		CoreISP3_I2C_Write(0xED27, 0x0011);
		CoreISP3_I2C_Write(0xEDE2, 0x0003);
 		CoreISP3_I2C_Write(0xE012, 0x0002);

		if(zoomRate > 2.0f)
		{
			CoreISP3_I2C_Write(0xEDE2, 0x0004);
		}
		#endif
		
		//startPosDiffx = (sensorOutputWidth - width)/2;
		//startPosDiffy = (sensorOutputHeight - height)/2;

		ISP3_Set_StillWindowsStartX(startPosDiffx);
		ISP3_Set_StillWindowsStartY(startPosDiffy);
		ISP3_Set_StillWindowsWidth(width);
		ISP3_Set_StillWindowsHeight(height);
		ISP3_Set_ScaleInputWindowsWidth(width);
		ISP3_Set_ScaleInputWindowsHeight(height);
		ISP3_Set_ScaleOutputWindowsWidth(ispOutputWidth); // Jacky change and test ok
		ISP3_Set_ScaleOutputWindowsHeight(ispOutputHeight);
		/*
		ISP3_Set_JpegWindowsWidth(ispOutputWidth);
		ISP3_Set_JpegWindowsHeight(ispOutputHeight);
		ISP3_Set_ThumbnailScaleInputWidth(ispOutputWidth);
		ISP3_Set_ThumbnailScaleInputHeight(ispOutputHeight);
		ISP3_Set_ScalePreviewWindowsInputWidth(ispOutputWidth);
		ISP3_Set_ScalePreviewWindowsInputHeight(ispOutputHeight);
		*/

	}

	ISP3_RegWrite_Partial(0xE010, 7, 7, 1);	// Sensor 1 Input off

	CoreISP3_Send_Command(ISP3_CMD_AE_ON);	// AE ON
	
	return zoomRate;
}
#endif

void ISP3_SetZoomRate(enIsp3ZoomRate rate)
{
	//Yulong_TestZoom();
	//ISP3_PreviewZoomScaleSet(240, 320, CAM_ZOOM_200X);  // Jacky for test
	//ISP3_PreviewZoomScaleSet(1280, 960, CAM_ZOOM_250X);  // Jacky for test
	//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_200X);  // Jacky for test
	//ISP3_SetDigitalZoom(g_curDigitalZoom);  // for HVGA 3:2 mode
	//return;

	printk( "====> %s, rate: %d \n", __func__, rate);
	g_curDigitalZoom = rate;
	
	switch(rate)
	{
		case enISP_ZOOM_RATE_1:
			//ISP3_SetDigitalZoomRate(1.0f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_100X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(ispOutputWidth, ispOutputHeight, CAM_ZOOM_100X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_1_25:
			//ISP3_SetDigitalZoomRate(1.25f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_125X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(ispOutputWidth, ispOutputHeight, CAM_ZOOM_125X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_1_5:
			//ISP3_SetDigitalZoomRate(1.5f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_150X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(ispOutputWidth, ispOutputHeight, CAM_ZOOM_150X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_1_75:
			//ISP3_SetDigitalZoomRate(1.75f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_200X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(ispOutputWidth, ispOutputHeight, CAM_ZOOM_200X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_2:
			//ISP3_SetDigitalZoomRate(2.0f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_250X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(ispOutputWidth, ispOutputHeight, CAM_ZOOM_250X);  // Jacky for test
			break;
		case enISP_ZOOM_RATE_2_5:
			//ISP3_SetDigitalZoomRate(2.5f);
			//ISP3_VGA_PreviewZoomScaleSet(640, 480, CAM_ZOOM_100X);  // Jacky for test
			ISP3_PreviewZoomScaleSet(ispOutputWidth, ispOutputHeight, CAM_ZOOM_100X);  // Jacky for test
			break;
		default:
			break;
		}
}
#endif


  
