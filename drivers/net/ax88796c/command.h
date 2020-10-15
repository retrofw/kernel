/*
 *********************************************************************************
 *     Copyright (c) 2005	ASIX Electronic Corporation      All rights reserved.
 *
 *     This is unpublished proprietary source code of ASIX Electronic Corporation
 *
 *     The copyright notice above does not evidence any actual or intended
 *     publication of such source code.
 *********************************************************************************
 */
 
#ifndef command_h
#define command_h

/* NAMING CONSTANT DECLARATIONS */
#define AX88796C_SIGNATURE	"AX88796C"

/* ioctl Command Definition */
#define AX_PRIVATE		SIOCDEVPRIVATE

/* private Command Definition */
#define AX_SIGNATURE			0
#define AX_CFG_TEST_PKT			1

typedef struct _AX_IOCTL_COMMAND {

	unsigned short	ioctl_cmd;
	unsigned char	sig[16];
	unsigned char	pattern;
	unsigned char	type;
		#define AX_PACKET_TYPE_RAND	1
		#define AX_PACKET_TYPE_FIX	2
	unsigned char	speed;
	unsigned short	length;
}AX_IOCTL_COMMAND;

#endif /* end of command.h */
