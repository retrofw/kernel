//
// Copyright (c) Ingenic Semiconductor Co., Ltd. 2007.
//

//#include <common.h>
//#include <command.h>
#include <asm/mipsregs.h>
//#include <asm/jz4740.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>
#include <linux/proc_fs.h>


#define I2CSim_READ_WRITE_RETRY_TIMES 5
#define SDA_PIN  (32 * 3 + 30)   //GPE12
#define SCL_PIN  (32 * 3 + 31)	 //GPE13	

#define SDA_HIGH  __gpio_set_pin(SDA_PIN)
#define SDA_LOW   __gpio_clear_pin(SDA_PIN)

#define SCL_HIGH  __gpio_set_pin(SCL_PIN)
#define SCL_LOW   __gpio_clear_pin(SCL_PIN)

#define SDA_DATA  __gpio_get_pin(SDA_PIN)

#define SDA_OUT	  __gpio_as_output(SDA_PIN)	
#define SDA_IN	  __gpio_as_input(SDA_PIN)	

//------------------------------------------------------------------------------
#if 0
extern PJZ4740_GPIO2	v_pGPIOReg;
#define	I2CSim_READ_WRITE_RETRY_TIMES		5
#define SDA_PIN			3*32 + 23
#define SCL_PIN			3*32 + 24
#define	SDA_HIGH		v_pGPIOReg->group[3].DATS = GPIO_23
#define	SDA_LOW			v_pGPIOReg->group[3].DATC = GPIO_23

#define	SCL_HIGH		v_pGPIOReg->group[3].DATS = GPIO_24
#define	SCL_LOW			v_pGPIOReg->group[3].DATC = GPIO_24

#define SDA_DATA		((v_pGPIOReg->group[3].PIN >> 23) &1)

#define SDA_OUT			v_pGPIOReg->group[3].DIRS = GPIO_23
#define SDA_IN			v_pGPIOReg->group[3].DIRC = GPIO_23	
#endif

typedef unsigned char BOOL;
typedef void VOID;

#define TRUE 1
#define FALSE 0

extern void mmdelay(int msec);

BOOL	g_bi2cInited = FALSE;

//-----------------------------------------------------------------------------

VOID I2CSim_Delay(volatile unsigned int delay)
{
	volatile unsigned int i;
	for ( i = 0; i < delay; i++ );
}
VOID I2CSim_Start( VOID )
{
	if(SDA_DATA == 0)
	{
		SDA_HIGH;
		I2CSim_Delay(500);
	}
	SDA_LOW;
	I2CSim_Delay(500);	
	SCL_LOW;
	I2CSim_Delay(500);
}
VOID I2CSim_Stop( VOID )
{
	SCL_HIGH;
}  
//------------------------------------------------------------------------------

void crypt_init()
{
	mmdelay(10);
	__gpio_as_output(SDA_PIN);
	__gpio_as_output(SCL_PIN);
	__gpio_enable_pull(SDA_PIN);	
	__gpio_enable_pull(SCL_PIN);	
	__gpio_set_pin(SDA_PIN);	
	__gpio_set_pin(SCL_PIN);


	//generate 5 PLUS on SDA
	int i;
	for(i=0; i<5; i++)
	{
		__gpio_set_pin(SDA_PIN);	
		mmdelay(5);
		__gpio_clear_pin(SDA_PIN);
		mmdelay(5);
	}		

	__gpio_set_pin(SDA_PIN);	
	__gpio_set_pin(SCL_PIN);
}	

VOID I2CSim_Init(VOID)
{
	if ( !g_bi2cInited )
	{
#if 0		
		GpioInit();
		v_pGPIOReg->group[3].PEC = GPIO_23;
		v_pGPIOReg->group[3].PEC = GPIO_24;
		SetGpioOutput(SDA_PIN, TRUE);
		SetGpioOutput(SCL_PIN, TRUE);
#endif
		__gpio_as_output(SDA_PIN);
		__gpio_as_output(SCL_PIN);
		__gpio_enable_pull(SDA_PIN);	
		__gpio_enable_pull(SCL_PIN);	
		__gpio_set_pin(SDA_PIN);	
		__gpio_set_pin(SCL_PIN);
		g_bi2cInited = TRUE;
	}
}

void crypt_reset()
{
	int i;
	
	__gpio_as_func0(SDA_PIN);
	__gpio_as_func0(SCL_PIN);
	__gpio_as_output(SDA_PIN);
	__gpio_as_output(SCL_PIN);
	__gpio_enable_pull(SDA_PIN);	
	__gpio_enable_pull(SCL_PIN);	

	__gpio_set_pin(SDA_PIN);	
	for(i=0; i<9; i++)
	{
		__gpio_set_pin(SCL_PIN);	
		mmdelay(5);
		__gpio_clear_pin(SDA_PIN);
		mmdelay(5);
	}		

	//create start
	__gpio_set_pin(SDA_PIN);	
	__gpio_set_pin(SCL_PIN);	
	
	mmdelay(5);
	
	__gpio_clear_pin(SDA_PIN);	

}

EXPORT_SYMBOL(crypt_reset);

//------------------------------------------------------------------------------

BOOL I2CSim_ReadByte(unsigned char* pucOut, BOOL bAck)
{
	unsigned char i;
	SDA_IN;
	//v_pGPIOReg->group[3].PEC = GPIO_23;
	REG_GPIO_PXPEC(1) = REG_GPIO_PXPEC(1) | 0x10000000; //GPB28
	I2CSim_Delay(100);
	*pucOut = 0;
	for ( i = 0; i < 8; i ++ )
	{
		SCL_HIGH;
		*pucOut |= (unsigned char)((SDA_DATA & 0x1) << (7-i));
		I2CSim_Delay(500);
		SCL_LOW;
		I2CSim_Delay(500);
	}
	if( bAck)
	{
		SDA_OUT;
		I2CSim_Delay(500);
		SDA_LOW;
		I2CSim_Delay(500);
		SCL_HIGH;
		I2CSim_Delay(500);
		SCL_LOW;
		I2CSim_Delay(500);	
		return ( TRUE );
		
	}
	return ( TRUE );
}

//------------------------------------------------------------------------------

BOOL I2CSim_WriteByte(unsigned char data)
{
	unsigned char i;
	for ( i = 0; i < 8; i ++ )
	{
		if(data & 0x80)
			SDA_HIGH;
		else 
			SDA_LOW;
		I2CSim_Delay(500);
		SCL_HIGH;
		I2CSim_Delay(500);
		SCL_LOW;
		I2CSim_Delay(500);	
		data = (data << 1);
	}
	SDA_IN;
	//v_pGPIOReg->group[3].PEC = GPIO_23;
	//REG_GPIO_PXPEC(3) = REG_GPIO_PXPEC(3) | 0x800000; //GPD23
	REG_GPIO_PXPEC(1) = REG_GPIO_PXPEC(1) | 0x10000000; //GPB28
	SCL_HIGH;
	I2CSim_Delay(500);
	i = 10;
	while(i)
	{
		if (!(SDA_DATA & 1))   //ACK received
		{
			SCL_LOW;
			I2CSim_Delay(500);	
			SDA_OUT;
			return ( TRUE );
		}
		SCL_LOW;
		I2CSim_Delay(500);
		SCL_HIGH;
		I2CSim_Delay(500);
		i--;
	}
	SCL_LOW;
	I2CSim_Delay(500);
	SDA_OUT;
	return ( FALSE );
}

//------------------------------------------------------------------------------

VOID I2CSim_Disable(VOID)
{
	SDA_HIGH;
	SCL_HIGH;
	g_bi2cInited = FALSE;
	//RETAILMSG(1, (TEXT("-L3Deinit\r\n")));
}
//------------------------------------------------------------------------------

BOOL I2CSim_CommonRead_crypto(unsigned char ucSlaveID, unsigned char* cmdBuffer, unsigned char cmdNumber,unsigned char* pucBuffer, unsigned char ucBytes)
{
	BOOL	bRet = FALSE;
	unsigned short	usRetryTimes = 0, usErr = 0;
	unsigned char	i;
	unsigned char *	pBuffer = pucBuffer;
	//================xltao add start======================
	unsigned char * pcmdBuffer = cmdBuffer;
	//================xltao add end========================

	if ( !pBuffer )
		return (FALSE);

CR_Retry:
	I2CSim_Init();
	pBuffer = pucBuffer;
  
  //===============xltao add start=======================
  pcmdBuffer = cmdBuffer;
  //===============xltao add end=========================

	if ( usRetryTimes > I2CSim_READ_WRITE_RETRY_TIMES )
		goto CR_Exit;
	I2CSim_Start();
	if ( !I2CSim_WriteByte(ucSlaveID) )
	{
		usErr = 1;
		goto CR_W_Err;
	}
	
	//====================xltao add start===================
	for(i = 0; i < cmdNumber; i++)
	{
		 	if ( !I2CSim_WriteByte(*(pcmdBuffer + i)) )
			{
				usErr = 3;
				goto CR_W_Err;
			}
			
	}
	//====================xltao add end=====================
	
	//SDA_IN;
	for ( i = 0; i < ucBytes; i ++ )
	{
		if ( !I2CSim_ReadByte(pBuffer, TRUE) )
	//	if ( !I2CSim_ReadByte(pBuffer, FALSE) )
		{
			usErr = 2;
			goto CR_R_Err;
		}
		pBuffer ++;
	}
	//SDA_OUT;
	I2CSim_Stop();
	I2CSim_Delay(500);
	I2CSim_Disable();
	I2CSim_Delay(500);
	bRet = TRUE;

	return (bRet);

CR_W_Err:
CR_R_Err:
	usRetryTimes ++;
	I2CSim_Stop();
	I2CSim_Delay(50);
	goto CR_Retry;

CR_Exit:
	I2CSim_Stop();
	I2CSim_Delay(50);
	I2CSim_Disable();
	//RETAILMSG(1, (TEXT("I2CSimLIB : I2C Read ERROR, Err = %d!\r\n"), usErr));
	return (FALSE);	
}


BOOL I2CSim_CommonRead(unsigned char ucSlaveID, unsigned char * pucBuffer, unsigned char ucBytes)
{
	BOOL	bRet = FALSE;
	unsigned short	usRetryTimes = 0, usErr = 0;
	unsigned char	i;
	unsigned char *	pBuffer = pucBuffer;


	if ( !pBuffer )
		return (FALSE);

CR_Retry:
	I2CSim_Init();
	pBuffer = pucBuffer;
  


	if ( usRetryTimes > I2CSim_READ_WRITE_RETRY_TIMES )
		goto CR_Exit;
	I2CSim_Start();
	if ( !I2CSim_WriteByte(ucSlaveID) )
	{
		usErr = 1;
		goto CR_W_Err;
	}
	
	
	//SDA_IN;
	for ( i = 0; i < ucBytes; i ++ )
	{
		if ( !I2CSim_ReadByte(pBuffer, TRUE) )
	//	if ( !I2CSim_ReadByte(pBuffer, FALSE) )
		{
			usErr = 2;
			goto CR_R_Err;
		}
		pBuffer ++;
	}
	//SDA_OUT;
	I2CSim_Stop();
	I2CSim_Delay(500);
	I2CSim_Disable();
	I2CSim_Delay(500);
	bRet = TRUE;

	return (bRet);

CR_W_Err:
CR_R_Err:
	usRetryTimes ++;
	I2CSim_Stop();
	I2CSim_Delay(50);
	goto CR_Retry;

CR_Exit:
	I2CSim_Stop();
	I2CSim_Delay(50);
	I2CSim_Disable();
	//RETAILMSG(1, (TEXT("I2CSimLIB : I2C Read ERROR, Err = %d!\r\n"), usErr));
	return (FALSE);	
}


//------------------------------------------------------------------------------

BOOL I2CSim_CommonWrite(unsigned char ucSlaveID, unsigned char* pucBuffer, unsigned char ucBytes)
{
	unsigned char i;
	unsigned short	usWriteRetryTimes = 0;
	//xltao add start
	unsigned short  errNumber = 0;
	//xltao add end
	unsigned char *	pBuffer = pucBuffer;

	if ( !pBuffer )
		return (FALSE);

CW_Retry:
	I2CSim_Init();
	pBuffer = pucBuffer;
	if ( usWriteRetryTimes > I2CSim_READ_WRITE_RETRY_TIMES )
		goto CW_Exit;
	I2CSim_Start();

	if ( !I2CSim_WriteByte(ucSlaveID) ){
		errNumber = 1; //xltao add start
		goto CW_Err;
	}

	for ( i = 0; i < ucBytes; i ++ )
	{
		if ( !I2CSim_WriteByte(*pBuffer) ){
			
			errNumber = 2;  //xltao add start
			goto CW_Err;
		 
		}
		pBuffer ++;
	}
	I2CSim_Stop();
	I2CSim_Delay(500);
	I2CSim_Disable();
	return (TRUE);

CW_Err:
	usWriteRetryTimes ++;
	goto CW_Retry;

CW_Exit:
	I2CSim_Stop();
	I2CSim_Delay(500);
	I2CSim_Disable();
	//RETAILMSG(1, (TEXT("I2CLIB : I2C Write ERROR: %d!\r\n"), errNumber));
	return (FALSE);
}
//------------------------------------------------------------------------------
