/*****************************************************************************/
/*  TRGMSG.C v6.0.13                                                          */
/*  Copyright (c) 1995-2007, 2010 Texas Instruments Incorporated                   */
/*****************************************************************************/

/*****************************************************************************/
/* Bottom level data transfer routines for host communication with the       */
/* target.                                                                   */
/*                                                                           */
/* Functions:                                                                */
/*  writemsg()  -  Sends the passed data and parameters on to the host.      */
/*  readmsg()   -  Reads the data and parameters passed from the host.       */
/*****************************************************************************/
#include <linux/types.h>
#include "linkage.h"
#include "trgcio.h"


#if BSSCIOBUF
#define DEFCIO(size) unsigned char _CIOBUF_[size]

#elif defined(_MVP_PP) || defined(_MVP_MP)
#pragma SHARED   (_CIOBUF_);
#pragma DATA_SECTION (_CIOBUF_, ".cio");
#pragma DATA_ALIGN   (_CIOBUF_, 4);
#define DEFCIO(size) volatile unsigned char _CIOBUF_[size]

#elif defined(_TMS320C6X)
#ifdef CONFIG_TI_C6X_COMPILER
#pragma DATA_SECTION (_CIOBUF_, ".cio");
#pragma DATA_ALIGN   (_CIOBUF_, 4);
#endif
#define DEFCIO(size) _DATA_ACCESS volatile unsigned char _CIOBUF_[size]

#elif defined(_TMS370C8)
#define STR(x) #x
#define DEFCIO(size) __asm("__CIOBUF_: .usect  .cio," STR(size)); \
		     extern far volatile unsigned char _CIOBUF_[size]

#elif defined(_TMS320C30) || defined(_TMS320C40)
#define STR(x) #x
#define DEFCIO(size) __asm("__CIOBUF_: .usect  .cio," STR(size)); \
                    extern volatile unsigned char _CIOBUF_[size]
#else
#define STR(x) #x
#define DEFCIO(size) __asm("__CIOBUF_: .usect  .cio," STR(size) ",4"); \
		     extern volatile unsigned char _CIOBUF_[size]
#endif

DEFCIO(CIOBUFSIZ);


/***************************************************************************/
/*                                                                         */
/*  WRITEMSG()  -  Sends the passed data and parameters on to the host.    */
/*                                                                         */
/***************************************************************************/
_CODE_ACCESS void writemsg(               unsigned char  command,
                           register const unsigned char *parm,
                           register const          char *data,
                                          unsigned int   length)
{
    register unsigned volatile char * p = (volatile unsigned char *) _CIOBUF_;

    register unsigned int i;

    /***********************************************************************/
    /* THE LENGTH IS WRITTEN AS A TARGET INT                               */
    /***********************************************************************/
    *(unsigned int *)p = length;
    p += sizeof(unsigned int);

    /***********************************************************************/
    /* THE COMMAND IS WRITTEN AS A TARGET BYTE                             */
    /***********************************************************************/
    *p++ = command;

    /***********************************************************************/
    /* PACK THE PARAMETERS AND DATA SO THE HOST READS IT AS BYTE STREAM    */
    /***********************************************************************/
    for (i = 0; i < 8; i++)      PACKCHAR(*parm++, p, i);
    for (i = 0; i < length; i++) PACKCHAR(*data++, p, i+8);


    /***********************************************************************/
    /* THE BREAKPOINT THAT SIGNALS THE HOST TO DO DATA TRANSFER            */
    /***********************************************************************/
    __asm("	.global	C$$IO$$");

    __asm("	nop");
    __asm("C$$IO$$:nop");
}


/***************************************************************************/
/*                                                                         */
/*  READMSG()   -  Reads the data and parameters passed from the host.     */
/*                                                                         */
/***************************************************************************/
_CODE_ACCESS void readmsg(register unsigned char *parm,
                          register char          *data)
{
    register unsigned volatile char * p = (volatile unsigned char *) _CIOBUF_;

    register unsigned int i;
    unsigned int length;

    /***********************************************************************/
    /* THE LENGTH IS READ AS A TARGET INT                                  */
    /***********************************************************************/
    length = *(unsigned int *)p;
    p += sizeof(unsigned int);

    /***********************************************************************/
    /* UNPACK THE PARAMETERS AND DATA                                      */
    /***********************************************************************/
    for (i = 0; i < 8; i++) *parm++ = UNPACKCHAR(p, i);
    if (data != NULL)
       for (i = 0; i < length; i++) *data++ = UNPACKCHAR(p, i+8);
}
