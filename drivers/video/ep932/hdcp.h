#ifndef HDCP_H
#define HDCP_H

#define HDCP_TIMER_PERIOD 					5		//   

// HDCP Transmiter Link State
typedef enum {
	A0_WAIT_FOR_ACTIVE_RX,
	A1_EXCHANGE_KSVS,
	A2_COMPUTATIONS,
	A3_VALIDATE_RECEIVER,
	A4_AUTHENTICATED,
	A5_LINK_INTEGRITY_CHECK,
	A6_TEST_FOR_REPEATER,
	A8_WAIT_FOR_READY,
	A9_READ_KSV_LIST
} hdcp_state_t;

#define HDCP_ERROR_BKSV								0x80
#define HDCP_ERROR_AKSV								0x40
#define HDCP_ERROR_R0								0x20
#define HDCP_ERROR_RI								0x10
#define HDCP_ERROR_REPEATERRDY							0x08
#define HDCP_ERROR_REPEATERSHA							0x04
#define HDCP_ERROR_RSEN								0x02
#define HDCP_ERROR_REPEATERMAX							0x01

extern hdcp_state_t hdcp_authentication_task(unsigned char receiverrdy);
extern void hdcp_stop(void);
extern unsigned char hdcp_get_status(void);
extern void hdcp_timer(void);
extern void hdcp_ext_ri_trigger(void);

// Special Functions
extern void hdcp_assign_rksv_list(unsigned char *prevocationlist, unsigned char listnumber);
extern void hdcp_fake(unsigned char enable);
extern void hdcp_extract_bksv_bcaps3(unsigned char *bksv_bcaps3);
extern void hdcp_extract_fifo(unsigned char *pfifo, unsigned char listnumber);
extern void hdcp_extract_sha_m0(unsigned char *sha_m0);

#endif // HDCP_H
