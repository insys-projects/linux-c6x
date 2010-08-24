/*****************************************************************************/
/* linkage.h   v6.0.13                                                        */
/* Copyright (c) 1998-2007 Texas Instruments Incorporated                    */
/*****************************************************************************/

#ifndef _LINKAGE
#define _LINKAGE

#if 1

#define _CODE_ACCESS
#define _DATA_ACCESS

#else
/*--------------------------------------------------------------------------*/
/* Define _CODE_ACCESS ==> how to call RTS functions                        */
/*--------------------------------------------------------------------------*/
#ifndef _FAR_RTS
#define _CODE_ACCESS
#elif _FAR_RTS == 0
#define _CODE_ACCESS near
#else
#define _CODE_ACCESS far
#endif

/*--------------------------------------------------------------------------*/
/* Define _DATA_ACCESS ==> how to access RTS global or static data          */
/*--------------------------------------------------------------------------*/
#define _DATA_ACCESS far

/*--------------------------------------------------------------------------*/
/* Define _IDECL ==> how inline functions are declared                      */
/*--------------------------------------------------------------------------*/
#ifdef _INLINE
#define _IDECL static __inline
#define _IDEFN static __inline
#else
#ifdef __cplusplus
#define _IDECL extern "C" _CODE_ACCESS
#else
#define _IDECL extern _CODE_ACCESS
#endif
#define _IDEFN _CODE_ACCESS
#endif

#endif

#endif /* ifndef _LINKAGE */
