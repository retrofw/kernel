/******************************************************************************\

          (c) Copyright Explore Semiconductor, Inc. Limited 2006
                           ALL RIGHTS RESERVED 

--------------------------------------------------------------------------------

 Please review the terms of the license agreement before using this file.
 If you are not an authorized user, please destroy this source code file  
 and notify Explore Semiconductor Inc. immediately that you inadvertently 
 received an unauthorized copy.  

--------------------------------------------------------------------------------

  File        :  type.h   

  Description :  Define the C-compiler variable syntax
                  (modified from Gordon's ICP10 souce code) 

\******************************************************************************/

#ifndef _TYPE_H
#define _TYPE_H


// =============================================================================
// =============================================================================

//typedef unsigned char BYTE;
//typedef BYTE *PBYTE;
//typedef unsigned int WORD;
//typedef unsigned int *PWORD;
//typedef unsigned long DWORD;
//typedef DWORD *PDWORD;

//typedef unsigned long ULONG;
//typedef unsigned long *PULONG;
//typedef unsigned short USHORT;
//typedef USHORT *PUSHORT;
//typedef unsigned char UCHAR;
//typedef UCHAR *PUCHAR;


// =============================================================================
// =============================================================================

//typedef void (*FVN)(void);

// =============================================================================
// =============================================================================

//#define FALSE               (0)
//#define TRUE                (!FALSE)
//#define OFF                 (0)
//#define ON                  (!OFF)
//#define ARRAYSIZE(ary)      (sizeof(ary)/sizeof(ary[0])) 

// =============================================================================
// =============================================================================

//for big endian 
//#define LOBYTE(x)           (BYTE)((x)>>8) 
//#define HIBYTE(x)           (BYTE)((x) & 0x00FF) 
//#define MKWORD(hi,lo)       (WORD)(((lo)<<8) | hi)                     

// =============================================================================
// =============================================================================

#define setb0(x)                (x |= 0x01)
#define setb1(x)                (x |= 0x02)
#define setb2(x)                (x |= 0x04)
#define setb3(x)                (x |= 0x08)
#define setb4(x)                (x |= 0x10)
#define setb5(x)                (x |= 0x20)
#define setb6(x)                (x |= 0x40)
#define setb7(x)                (x |= 0x80)

#define clrb0(x)                (x &= 0xFE)
#define clrb1(x)                (x &= 0xFD)
#define clrb2(x)                (x &= 0xFB)
#define clrb3(x)                (x &= 0xF7)
#define clrb4(x)                (x &= 0xEF)
#define clrb5(x)                (x &= 0xDF)
#define clrb6(x)                (x &= 0xBF)
#define clrb7(x)                (x &= 0x7F)

#define getb0(x)                (x & 0x01)
#define getb1(x)                ((x & 0x02)>>1)
#define getb2(x)                ((x & 0x04)>>2)
#define getb3(x)                ((x & 0x08)>>3)
#define getb4(x)                ((x & 0x10)>>4)
#define getb5(x)                ((x & 0x20)>>5)
#define getb6(x)                ((x & 0x40)>>6)
#define getb7(x)                ((x & 0x80)>>7)


// =============================================================================
// =============================================================================

// math
#ifndef min
#define min(a,b) (((a)<(b))? (a):(b))
#endif

#ifndef max
#define max(a,b) (((a)>(b))? (a):(b))
#endif

#endif
