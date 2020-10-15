//
// Copyright (c) Ingenic Semiconductor Co., Ltd. 2007.
//

#include <asm/jzsoc.h>


#define	I2CSim_READ_WRITE_RETRY_TIMES		5
#define SDA_PIN			(5*32 + 17)
#define SCL_PIN			(5*32 + 16)


#define __gpio_out_to_in(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXDIRC(p) = (1 << (o));		\
} while (0)

#define __gpio_in_to_out(n)			\
do {						\
	unsigned int p, o;			\
	p = (n) / 32;				\
	o = (n) % 32;				\
	REG_GPIO_PXDIRS(p) = (1 << (o));		\
} while (0)

#define	SDA_HIGH		__gpio_set_pin(SDA_PIN)
#define	SDA_LOW			__gpio_clear_pin(SDA_PIN)

#define	SCL_HIGH		__gpio_set_pin(SCL_PIN)
#define	SCL_LOW			__gpio_clear_pin(SCL_PIN)

#define SDA_DATA		__gpio_get_pin(SDA_PIN)

#define SDA_OUT                 __gpio_in_to_out(SDA_PIN)
#define SDA_IN                  __gpio_out_to_in(SDA_PIN)
#define SCL_OUT                 __gpio_in_to_out(SCL_PIN)

#define SDA_OUT_INIT			__gpio_as_output(SDA_PIN)
#define SDA_IN_INIT			__gpio_as_input(SDA_PIN)
#define SCL_OUT_INIT                 __gpio_as_output(SCL_PIN)

#define TRUE	1
#define FALSE	0

unsigned char	g_bi2cInited = FALSE;
//-----------------------------------------------------------------------------

void I2CSim_Delay(volatile unsigned int delay)
{
	volatile unsigned int i;
	for ( i = 0; i < delay; i++ );
}
void I2CSim_Start( void )
{
	SDA_HIGH;
	I2CSim_Delay(500);
	SCL_HIGH;
	I2CSim_Delay(500);
	SDA_LOW;
	I2CSim_Delay(500);	
	SCL_LOW;
	I2CSim_Delay(500);
}
void I2CSim_Stop( void )
{
	SCL_HIGH;
}  
//------------------------------------------------------------------------------

void I2CSim_Init(void)
{
	if ( !g_bi2cInited )
	{
		SDA_OUT_INIT;
		SCL_OUT_INIT;
		g_bi2cInited = TRUE;
	}
}

//------------------------------------------------------------------------------

char I2CSim_ReadByte(unsigned char * pucOut, char bAck)
{
	unsigned char	i;

	SDA_IN;
	I2CSim_Delay(100);
	*pucOut = 0;
	for ( i = 0; i < 8; i ++ )
	{
		SCL_HIGH;
		*pucOut |= (unsigned char )((SDA_DATA & 0x1) << (7-i));
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

char I2CSim_WriteByte(unsigned char data)
{
	unsigned char	i;

	for ( i = 0; i < 8; i ++ )
	{
		if(data & 0x80)
			SDA_HIGH;
		else 
			SDA_LOW;
		I2CSim_Delay(200);
		SCL_HIGH;
		I2CSim_Delay(200);
		SCL_LOW;
		I2CSim_Delay(200);	
		data = (data << 1);
	}
	SDA_IN;
	SCL_HIGH;
	I2CSim_Delay(200);
	i = 10;
	while(i)
	{
		if (!(SDA_DATA & 1))
		{
			SCL_LOW;
			I2CSim_Delay(200);	
			SDA_OUT;
			return ( TRUE );
		}
		SCL_LOW;
		I2CSim_Delay(200);
		SCL_HIGH;
		I2CSim_Delay(200);
		i--;
	}
	SCL_LOW;
	I2CSim_Delay(200);
	SDA_OUT;
	return ( FALSE );
}

//------------------------------------------------------------------------------

void I2CSim_Disable(void)
{
	SDA_HIGH;
	SCL_HIGH;
	g_bi2cInited = FALSE;
}

//------------------------------------------------------------------------------

unsigned char I2CSim_Read8(unsigned char ucSlaveID, char address)
{
	unsigned char	usRetryTimes = 0, usErr = 0;
	unsigned char	value=0;

CR_Retry:
	I2CSim_Init();

	if ( usRetryTimes > I2CSim_READ_WRITE_RETRY_TIMES )
		goto CR_Exit;
	I2CSim_Start();
	if ( !I2CSim_WriteByte(ucSlaveID) )
	{
		usErr = 1;
		goto CR_W_Err;
	}
	if ( !I2CSim_WriteByte(address) )
	{
		usErr = 2;
		goto CR_W_Err;
	}
	I2CSim_Start();
	if ( !I2CSim_WriteByte(ucSlaveID|1) )
        {
                usErr = 3;
                goto CR_W_Err;
        }
	if ( !I2CSim_ReadByte(&value, TRUE) )
	{
		usErr = 4;
		goto CR_R_Err;
	}

	I2CSim_Stop();
	I2CSim_Delay(500);
	I2CSim_Disable();
	I2CSim_Delay(500);


	return (value);

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
	printk("I2CSimLIB : I2C Read ERROR, Err = %d!\r\n", usErr);
	return (value);	
}

//------------------------------------------------------------------------------

void I2CSim_Write8(unsigned char ucSlaveID, char address ,char value)
{
	unsigned short	usWriteRetryTimes = 0;

CW_Retry:
	I2CSim_Init();

	if ( usWriteRetryTimes > I2CSim_READ_WRITE_RETRY_TIMES )
		goto CW_Exit;
	I2CSim_Start();

	if ( !I2CSim_WriteByte(ucSlaveID) )
		goto CW_Err;

	if ( !I2CSim_WriteByte(address) )
		goto CW_Err;
		
	if ( !I2CSim_WriteByte(value) )
		goto CW_Err;
		
	I2CSim_Stop();
	I2CSim_Delay(500);
	I2CSim_Disable();
	return ;

CW_Err:
	usWriteRetryTimes ++;
	goto CW_Retry;

CW_Exit:
	I2CSim_Stop();
	I2CSim_Delay(500);
	I2CSim_Disable();
	printk("I2CLIB : I2C Write ERROR!\r\n");
}
//------------------------------------------------------------------------------




#define	SDA_HIGH		__gpio_set_pin(SDA_PIN)
#define	SDA_LOW			__gpio_clear_pin(SDA_PIN)

#define	SCL_HIGH		__gpio_set_pin(SCL_PIN)
#define	SCL_LOW			__gpio_clear_pin(SCL_PIN)

#define SDA_DATA		__gpio_get_pin(SDA_PIN)

#define SDA_OUT     __gpio_in_to_out(SDA_PIN)
#define SDA_IN      __gpio_out_to_in(SDA_PIN)
#define SCL_OUT     __gpio_in_to_out(SCL_PIN)

#define SDA_OUT_INIT		__gpio_as_output(SDA_PIN)
#define SDA_IN_INIT			__gpio_as_input(SDA_PIN)
#define SCL_OUT_INIT    __gpio_as_output(SCL_PIN)

#define TRUE	1
#define FALSE	0
#define GPIO_IIC_JACKY    // Jacky for test                                                  
                                                                                             
#ifdef GPIO_IIC_JACKY                                                                        
                                                                                             
#define SET_SCCB_DATA_HIGH  SDA_HIGH    // Jacky for test                                    
#define SET_SCCB_DATA_LOW   SDA_LOW    // Jacky for test                                   
#define SET_SCCB_CLK_HIGH   SCL_HIGH      // Jacky for test                                 
#define SET_SCCB_CLK_LOW    SCL_LOW       // Jacky for test                                 
#define SET_SCCB_DATA_INPUT    SDA_IN       // Jacky for test                        
#define SET_SCCB_DATA_OUTPUT   SDA_OUT       // Jacky for test                      
#define GET_SCCB_DATA_BIT    SDA_DATA                          
                                                                                             
#define SET_SCCB_CLK_OUTPUT  	SCL_OUT_INIT           // SDA_OUT_Mode(); 	// Jacky for test
                                                                                             
#define SENSOR_I2C_DELAY    (250)   // 80KHz for iic speed.... 100 92KHz                     
   
#define  I2C_READ       0x01
#define  READ_CMD       1
#define  WRITE_CMD      0
                                                                                       
#define I2C_START_TRANSMISSION \                                                             
{ \                                                                                          
    volatile unsigned char j; \
    SDA_OUT_INIT;	\
    SCL_OUT_INIT;	\                                                                         
    SET_SCCB_CLK_OUTPUT; \                                                                   
    SET_SCCB_DATA_OUTPUT; \                                                                  
    SET_SCCB_CLK_HIGH; \                                                                     
    SET_SCCB_DATA_HIGH; \                                                                    
    for (j = 0; j < SENSOR_I2C_DELAY; j++); \                                                
    SET_SCCB_DATA_LOW; \                                                                     
    for (j = 0; j < SENSOR_I2C_DELAY; j++); \                                                
    SET_SCCB_CLK_LOW; \                                                                      
}                                                                                            
                                                                                             
#define I2C_STOP_TRANSMISSION \                                                              
{ \                                                                                          
    volatile unsigned char j; \                                                                         
    SET_SCCB_CLK_OUTPUT; \                                                                   
    SET_SCCB_DATA_OUTPUT; \                                                                  
    SET_SCCB_CLK_LOW; \                                                                      
    SET_SCCB_DATA_LOW; \                                                                     
    for (j = 0; j < SENSOR_I2C_DELAY; j++); \                                                
    SET_SCCB_CLK_HIGH; \                                                                     
    for (j = 0; j < SENSOR_I2C_DELAY; j++); \                                                
    SET_SCCB_DATA_HIGH; \                                                                    
}                                                                                            
                                                                                             
static void SCCB_send_byte(unsigned char send_byte)                                                     
{                                                                                            
    volatile signed char i;                                                                  
    volatile unsigned char j;                                                                           
                                                                                             
    for (i = 7; i >= 0; i--) { /* data bit 7~0 */                                            
        if (send_byte & (1 << i)) {                                                          
            SET_SCCB_DATA_HIGH;                                                              
        }else {                                                                              
            SET_SCCB_DATA_LOW;                                                               
        }                                                                                    
                                                                                             
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
        SET_SCCB_CLK_HIGH;                                                                   
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
        SET_SCCB_CLK_LOW;                                                                    
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
    }                                                                                        
    /* don't care bit, 9th bit */                                                            
    SET_SCCB_DATA_LOW;                                                                       
    SET_SCCB_DATA_INPUT;                                                                     
    SET_SCCB_CLK_HIGH;                                                                       
    for (j = 0; j < SENSOR_I2C_DELAY; j++);                                                  
    SET_SCCB_CLK_LOW;                                                                        
    SET_SCCB_DATA_OUTPUT;                                                                    
}   /* SCCB_send_byte() */                                                                   
                                                                                             
                                                                                             
static unsigned char SCCB_get_byte(void)                                                                
{                                                                                            
    volatile signed char i;                                                                  
    volatile unsigned char j;                                                                           
    unsigned char get_byte = 0;                                                                         
                                                                                             
    SET_SCCB_DATA_INPUT;                                                                     
                                                                                             
    for (i = 7; i >= 0; i--) { /* data bit 7~0 */                                            
        SET_SCCB_CLK_HIGH;                                                                   
		for (j = 0; j < SENSOR_I2C_DELAY; j++);                                                  
        if (GET_SCCB_DATA_BIT)                                                               
            get_byte |= (1 << i);                                                            
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
        SET_SCCB_CLK_LOW;                                                                    
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
    }                                                                                        
    /* don't care bit, 9th bit */                                                            
    SET_SCCB_DATA_HIGH;                                                                      
    SET_SCCB_DATA_OUTPUT;                                                                    
    for (j = 0; j < SENSOR_I2C_DELAY; j++);                                                  
    SET_SCCB_CLK_HIGH;                                                                       
    for (j = 0; j < SENSOR_I2C_DELAY; j++);                                                  
    SET_SCCB_CLK_LOW;                                                                        
                                                                                             
    return get_byte;                                                                         
}   /* SCCB_get_byte() */                                                                    
                                                                                             
#endif                                                                                       
                                                                                             
                                                                                             
unsigned short I2C_ReadControl(unsigned char IICID, unsigned short regaddr)       // dhlee @ 2005.10.08 support 2Byte read    
{                                                                                            
    unsigned short             readdata1 = 0, readdata = 0;                                             
    unsigned char              returnack;                                                               
	                                                                                           
#ifdef GPIO_IIC_JACKY                                                                        
         volatile unsigned char j;                                                                      
                                                                                             
        I2C_START_TRANSMISSION;                                                              
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
	SCCB_send_byte(IICID&(~I2C_READ));                                                         
                                                                                             
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
        SCCB_send_byte(regaddr);                                                             
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
                                                                                             
        I2C_START_TRANSMISSION;                                                              
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
        SCCB_send_byte(IICID |I2C_READ);                                                     
		                                                                                         
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
        readdata = SCCB_get_byte();                                                          
        for (j = 0; j < SENSOR_I2C_DELAY; j++);                                              
        I2C_STOP_TRANSMISSION;                                                               
#else                                                                                        
                                                                                             
    I2C_PortInitial();                                                                       
    I2C_StartCondition();                                                                    
     SWI2C_TIMER_Wait_us(SLOW_IIC_4);	                                                       
    I2C_SlaveAddress(IICID, WRITE_CMD);                                                      
    returnack = I2C_CheckAck();                                                              
    SWI2C_TIMER_Wait_us(SLOW_IIC_10);                                                        
                                                                                             
    I2C_RegisterAddress(regaddr);                                                            
    returnack = I2C_CheckAck();                                                              
    TIMER_Wait_us(1);                                                                        
                                                                                             
    I2C_StartCondition();                                                                    
    I2C_SlaveAddress(IICID, READ_CMD);                                                       
    returnack = I2C_CheckAck();                                                              
    TIMER_Wait_us(1);                                                                        
                                                                                             
    if (SI2CRWCNTL & 0x0002)                                                                 
    {                                                                                        
        readdata1 = I2C_ReadData();                                                          
        I2C_SendAck();                                                                       
        TIMER_Wait_us(1);                                                                    
    }                                                                                        
                                                                                             
    readdata = (unsigned short)((readdata1 << 8) | I2C_ReadData());                                     
                                                                                             
    I2C_CheckNotAck();                                                                       
    SWI2C_TIMER_Wait_us(4);                                                                  
    I2C_StopCondition();                                                                     
                                                                                             
    SWI2C_TIMER_Wait_us(SLOW_IIC_4);                                                         
#endif	                                                                                     
    return readdata;                                                                         
}  
unsigned short I2C_WriteControl(unsigned char IICID, unsigned short regaddr, unsigned short data)
{
   unsigned char              returnack = TRUE; 
   
#ifdef GPIO_IIC_JACKY

         volatile unsigned char j;

//	printk("%s %d \n",__FUNCTION__,__LINE__);
        I2C_START_TRANSMISSION;
//	printk("%s %d \n",__FUNCTION__,__LINE__);
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SCCB_send_byte(IICID&(~I2C_READ));
//  	printk("%s %d \n",__FUNCTION__,__LINE__);
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SCCB_send_byte(regaddr);
//	printk("%s %d \n",__FUNCTION__,__LINE__);
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        SCCB_send_byte(data);
//	printk("%s %d \n",__FUNCTION__,__LINE__);
        for (j = 0; j < SENSOR_I2C_DELAY; j++);
        I2C_STOP_TRANSMISSION;
//	printk("%s %d \n",__FUNCTION__,__LINE__);
#else

    I2C_StartCondition();
    SWI2C_TIMER_Wait_us(SLOW_IIC_4);
    I2C_SlaveAddress(IICID, WRITE_CMD);

    if (!I2C_CheckAck()) returnack = FALSE;

    SWI2C_TIMER_Wait_us(SLOW_IIC_4);
    I2C_RegisterAddress( regaddr );

    if (!I2C_CheckAck()) returnack = FALSE;

    SWI2C_TIMER_Wait_us(SLOW_IIC_4);
    I2C_WriteData( data );

    if (!I2C_CheckAck()) returnack = FALSE;

    SWI2C_TIMER_Wait_us(SLOW_IIC_4);  
    I2C_StopCondition();
    SWI2C_TIMER_Wait_us(SLOW_IIC_4);
#endif 

    return returnack;
}


 



                                                                                          
                                                                                             
