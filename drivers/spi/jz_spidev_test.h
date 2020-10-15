#ifndef __JZ_SPI_SYS_H__
#define __JZ_SPI_SYS_H__

#define JZSPI_MAJOR  249
#define JZSPI_NAME  "jz-spi"

//ioctl cmd
#define SET_SPI                1
#define WRITE_SPI           2
#define READ_SPI             3
#define SPI_SEL_CE           4
#define LOOP_SPI             5
#define GET_SPI_STATE  6
#define SET_SPI_MODE  7
#define SET_LSB_MSB  8

#define SPI_REGS_PRINT			9
#define	TEST_SPI_READ			10
#define TEST_SPI_WRITE			11
#define TEST_SPI_INIT			12
#define TEST_SPI_REGS_SET		13
#define TEST_SPI_STATUS			14
#define TEST_SPI_PIN_CFG		15
#define TEST_SPI_EN				16
#define TEST_SPI_CLOCK			17
#define	TEST_SPI_THRESHOLD_INT	18
#define TEST_SPI_READ_SUBSYSTEM		19
#define TEST_SPI_WRITE_SUBSYSTEM	20
#define TEST_SPI_MALLOC				21
#define TEST_SPI_DEVICE_CONFIG		22
#define TEST_SPI_TRANSFER_CONFIG	23
#define TEST_SPI_DMA_WRITE			24
#define TEST_SPI_FILE_WRITE			25
#define TEST_SPI_CHIPSELECT			26
#define TEST_SPI_SEND_BYTES			27
#define TEST_SPI_STANDARD_OPS		28


#define SPI_CPU_MODE	0
#define SPI_DMA_MODE	1

#define PIN_SSI_CLK		0
#define PIN_SSI_CE		1
#define PIN_SSI_CE2		2
#define PIN_SSI_GPC		3
#define PIN_SSI_DT		4
#define PIN_SSI_DR		5


			
typedef enum {
    IN_MODE,
    OUT_MODE
}SPI_MODE_ENUM;

typedef struct {
    int io;
    unsigned short val;
}SPI_PARAM_STRUCT;

// add by shumb
typedef struct {
	unsigned char chnl;
	unsigned char ptr;
    int io;
    unsigned int val;
	unsigned int reg[7];
	unsigned char * src;
	unsigned char * dest;
}T_SPI_PARAM_STRUCT;


typedef struct {
    int io;
    unsigned char burst_mode;
}IRQ_INFO_STRUCT;

typedef struct {
    int io;
    unsigned int txfifo_count;
    unsigned int rxfifo_count;
    unsigned char transfer_end;
    unsigned char is_busy;

    unsigned char txfifo_full;
    unsigned char rxfifo_empty;
    unsigned char rxfifo_half_full;
    unsigned char txfifo_half_empty;

    unsigned char underrun;
    unsigned char overrun;


}SPI_STATE_STRUCT;
#endif /* __JZ_SPI_SYS_H__ */

