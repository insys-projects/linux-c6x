/*****************************************************************************/
/*  TRGDRV.C v6.0.13                                                          */
/*  Copyright (c) 1996-2007 Texas Instruments Incorporated                   */
/*****************************************************************************/

/****************************************************************************/
/* Functions:                                                               */
/*    HOSTopen()    -    Sends open command with arguments to the host      */
/*    HOSTclose()   -    Sends close command with arguments to the host     */
/*    HOSTread()    -    Sends read command with arguments to the host      */
/*    HOSTwrite()   -    Sends write command with arguments to the host     */
/*    HOSTlseek()   -    Sends lseek command with arguments to the host     */
/*    HOSTunlink()  -    Sends unlink command with arguments to the host    */
/*    HOSTrename()  -    Sends rename command with arguments to the host    */
/*    GETENV()      -    Get the enviroment value for the passed variable   */
/*                        name                                              */
/*    HOSTTIME()    -    Get the time from the host                         */
/*    HOSTCLK()     -    Get the target clock value (CLK register)          */
/****************************************************************************/
#include <linux/types.h>
#include "linkage.h"
#include "_lock.h"

#include "trgdrv.h"
#include "trgcio.h"

#if defined(REDUCE_GETENV_BUFSIZE)
#define GETENV_VALUE_BUFSIZE 128
#else
#define GETENV_VALUE_BUFSIZE 200
#endif

static _DATA_ACCESS unsigned char parmbuf[8];

/****************************************************************************/
/* HOSTOPEN()  -  Pass the open command and its arguments to the host.      */
/****************************************************************************/
_CODE_ACCESS int HOSTopen(const char *path, unsigned flags, int llv_fd)
{
   int dev_fd;
   _lock();

   LOADSHORT(parmbuf,llv_fd,0);
   LOADSHORT(parmbuf,flags,2);
   writemsg(_DTOPEN,parmbuf,(char *)path,strlen(path)+1);
					 /* SEND NULL ACROSS ALSO */
   readmsg(parmbuf,NULL);

   dev_fd = UNLOADSHORT(parmbuf,0);
   _unlock();
   return (dev_fd < 0) ? dev_fd : llv_fd;
}

/****************************************************************************/
/* HOSTCLOSE()  -  Pass the close command and its arguments to the host.    */
/****************************************************************************/
_CODE_ACCESS int HOSTclose(int dev_fd)
{
   int result;
   _lock();

   LOADSHORT(parmbuf,dev_fd,0);

   writemsg(_DTCLOSE,parmbuf,NULL,0);
   readmsg(parmbuf,NULL);

   result = UNLOADSHORT(parmbuf,0);
   _unlock();
   return result;
}

/****************************************************************************/
/* HOSTREAD()  -  Pass the read command and its arguments to the host.      */
/****************************************************************************/
_CODE_ACCESS int HOSTread(int dev_fd, char *buf, unsigned count)
{
   int result;
   _lock();

   if (count > BUFSIZ) count = BUFSIZ;

   LOADSHORT(parmbuf,dev_fd,0);
   LOADSHORT(parmbuf,count,2);

   writemsg(_DTREAD,parmbuf,NULL,0);
   readmsg(parmbuf,buf);

   result = UNLOADSHORT(parmbuf,0);
   _unlock();
   return result;
}

/****************************************************************************/
/* HOSTWRITE()  -  Pass the write command and its arguments to the host.    */
/****************************************************************************/
_CODE_ACCESS int HOSTwrite(int dev_fd, const char *buf, unsigned count)
{
   int result;
   _lock();

   if (count > BUFSIZ) count = BUFSIZ;

   LOADSHORT(parmbuf,dev_fd,0);
   LOADSHORT(parmbuf,count,2);
   writemsg(_DTWRITE,parmbuf,(char *)buf,count);
   readmsg(parmbuf,NULL);

   result = UNLOADSHORT(parmbuf,0);
//   if (result <= 0)
	result = count;

   _unlock();
   return result;
}

/****************************************************************************/
/* HOSTLSEEK()  -  Pass the lseek command and its arguments to the host.    */
/****************************************************************************/
_CODE_ACCESS off_t HOSTlseek(int dev_fd, off_t offset, int origin)
{
   off_t result;
   _lock();

   LOADSHORT(parmbuf,dev_fd,0);
   LOAD32(parmbuf,offset,2);
   LOADSHORT(parmbuf,origin,6);

   writemsg(_DTLSEEK,parmbuf,NULL,0);
   readmsg(parmbuf,NULL);

   result = UNLOAD32(parmbuf,0);
   _unlock();
   return result;
}

_CODE_ACCESS int HOSTunlink(const char *path)
{
   int result;
   _lock();

   writemsg(_DTUNLINK,parmbuf,(char *)path,strlen(path) + 1);
   readmsg(parmbuf,NULL);

   result = UNLOADSHORT(parmbuf,0);
   _unlock();
   return result;
}

_CODE_ACCESS int HOSTrename(const char *old, const char *new)
{
   char combined[100];
   int  length;
   int  result;
   _lock();

   strcpy(combined,old);
   length = strlen(old)+1;
   strcpy(combined+length,new);
   length += strlen(new) + 1;

   writemsg(_DTRENAME,parmbuf,combined,length);
                                                 /*SEND NULL ACROSS ALSO*/
   readmsg(parmbuf,NULL);

   result = UNLOADSHORT(parmbuf,0);
   _unlock();
   return result;
}

/****************************************************************************/
/* GETENV()  -  Get the enviroment value for the passed variable name       */
/****************************************************************************/
_CODE_ACCESS char *getenv(const char *_string)
{
   static _DATA_ACCESS char result[GETENV_VALUE_BUFSIZE];

   _lock();

   writemsg(_DTGETENV,parmbuf,(char *)_string,strlen(_string) + 1);
   readmsg(parmbuf,result);

   _unlock();
   return strlen(result) ? result : 0;
}

/****************************************************************************/
/* HOSTTIME()  -  Get the time from the host                                */
/****************************************************************************/
_CODE_ACCESS time_t HOSTtime(void)
{
   time_t result;
   _lock();

   writemsg(_DTGETTIME,parmbuf,NULL,0);
   readmsg(parmbuf, NULL);

   result = (time_t)(UNLOAD32(parmbuf,0));
   _unlock();
   return result;
}

/****************************************************************************/
/* HOSTclock()  -  Get the current number of clock ticks                    */
/****************************************************************************/
_CODE_ACCESS clock_t HOSTclock(void)
{
   clock_t result;
   _lock();

   writemsg(_DTGETCLK,parmbuf,NULL,0);
   readmsg(parmbuf, NULL);

   result = (clock_t)(UNLOAD32(parmbuf,0));
   _unlock();
   return result;
}
