/*****************************************************************************/
/*  TRGCIO.H v6.0.13                                                          */
/*  Copyright (c) 1995-2007 Texas Instruments Incorporated                   */
/*****************************************************************************/

/*****************************************************************************/
/*  This file contains OTIS-driven CIO constant definitions                  */
/*****************************************************************************/

/*---------------------------------------------------------------------------*/
/* constants for the data transfer functions for the predefined driver       */
/*---------------------------------------------------------------------------*/
#define _DTOPEN    (0xF0)
#define _DTCLOSE   (0xF1)
#define _DTREAD    (0xF2)
#define _DTWRITE   (0xF3)
#define _DTLSEEK   (0xF4)
#define _DTUNLINK  (0xF5)
#define _DTGETENV  (0xF6)
#define _DTRENAME  (0xF7)
#define _DTGETTIME (0xF8)
#define _DTGETCLK  (0xF9)
#define _DTSYNC    (0xFF)

#define CIOBUFSIZ BUFSIZ+32


#define BUFSIZ      256


/*---------------------------------------------------------------------------*/
/* THESE MACROS PACK AND UNPACK SHORTS AND LONGS INTO CHARACTER ARRAYS       */
/*                                                                           */
/* THIS IS DONE BYTE REVERSED TO MAKE THE PC'S JOB EASIER AND SINCE THE      */
/*     DEVICE COULD BE BIG OR LITTLE ENDIAN                                  */
/*---------------------------------------------------------------------------*/
#define LOADSHORT(x,y,z)  { x[(z)]   = (unsigned short) (y); \
                            x[(z)+1] = (unsigned short) (y) >> 8;  }

#define UNLOADSHORT(x,z) ((short) ( (short) x[(z)] +             \
				   ((short) x[(z)+1] << 8)))

#define LOAD32(x,y,z) { x[(z)]   = (unsigned int) (y); \
                          x[(z)+1] = (unsigned int) (y) >> 8; \
                          x[(z)+2] = (unsigned int) (y) >> 16;  \
                          x[(z)+3] = (unsigned int) (y) >> 24;   }

#define UNLOAD32(x,z) ( (int) x[(z)]           +  \
		       ((int) x[(z)+1] << 8)   +  \
		       ((int) x[(z)+2] << 16)  +  \
		       ((int) x[(z)+3] << 24))

/*---------------------------------------------------------------------------*/
/* THESE MACROS CAN BE USED TO PACK AND UNPACK CHARACTER STREAMS INTO TARGET */
/* WORDS FOR MORE EFFICIENT TRANSFER.                                        */
/*---------------------------------------------------------------------------*/
#define PACKCHAR(val, base, byte) ( (base)[(byte)] = (val) )

#define UNPACKCHAR(base, byte)    ( (base)[byte] )

#define BSSCIOBUF 0


