/*****************************************************************************/
/*  TRGDRV.H v6.0.13                                                          */
/*  Copyright (c) 1996-2007 Texas Instruments Incorporated                   */
/*****************************************************************************/

extern _CODE_ACCESS int HOSTopen(const char *path, unsigned flags, int llv_fd),
                        HOSTclose(int dev_fd),
                        HOSTread(int dev_fd, char *buf, unsigned count),
                        HOSTwrite(int dev_fd, const char *buf, unsigned count),
                        HOSTunlink(const char *path),
                        HOSTrename(const char *old_name, const char *new_name);

extern _CODE_ACCESS off_t HOSTlseek(int dev_fd, off_t offset, int origin);
extern _CODE_ACCESS time_t HOSTtime(void);
extern _CODE_ACCESS clock_t HOSTclock(void);

extern _CODE_ACCESS void readmsg(register unsigned char *parm,
				 register char *data);

extern _CODE_ACCESS void writemsg(unsigned char  command,
                                  register const unsigned char *parm,
                                  register const          char *data,
                                  unsigned int   length);

