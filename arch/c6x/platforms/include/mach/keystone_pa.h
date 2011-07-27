/*
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef __MACH_C6X_KEYSTONE_PA_H
#define __MACH_C6X_KEYSTONE_PA_H

#define DEVICE_PA_BASE				0x02000000
#define DEVICE_PA_RUN_CHECK_COUNT		100
#define DEVICE_PA_NUM_PDSPS			6

#define PA_CMD_SIZE 16
#define PA_MAGIC_ID				0x0CEC11E0
#define PA_REG_MAILBOX_SLOT(pdsp, slot)		(0x00 + ((pdsp) * 0x10) + ((slot) * 0x04))
#define PA_REG_PDSP_CTL(pdsp)			(0x1000 + ((pdsp) * 0x100))
#define PA_REG_TIMER_CTL(pdsp)			(0x3000 + ((pdsp) * 0x100))
#define PA_REG_TIMER_LOAD(pdsp)			(0x3000 + ((pdsp) * 0x100) + 0x4)
#define PA_REG_TIMER_VAL(pdsp)			(0x3000 + ((pdsp) * 0x100) + 0x8)
#define PA_REG_TIMER_TIMER_IRQ(pdsp)	        (0x3000 + ((pdsp) * 0x100) + 0xC)
#define PA_MEM_PDSP_IRAM(pdsp)			(0x10000 + ((pdsp) * 0x8000))
#define PA_REG_PKTID_REV	                0x00400
#define PA_REG_PKTID_SOFT_RESET	                0x00404
#define PA_REG_PKTID_RANGE_LIM	                0x00408
#define PA_REG_PKTID_IDVAL	                0x0040C
#define PA_REG_LUT2_REV	                        0x00500
#define PA_REG_LUT2_SOFT_RESET	                0x00504
#define PA_REG_LUT2_DATA0	                0x00520
#define PA_REG_LUT2_ADD_DEL_KEY	                0x00530
#define PA_REG_LUT2_ADD_CTL	                0x00534
#define PA_REG_STATS_REV	                0x06000
#define PA_REG_STATS_SOFT_RESET	                0x06004
#define PA_REG_STATS_INCREM	                0x06008
#define PA_REG_STATS_CAPT	                0x0600C
#define PA_REG_STATS0 	                        0x06020

/* The pdsp control register */
#define PA_REG_VAL_PDSP_CTL_DISABLE_PDSP	1
#define PA_REG_VAL_PDSP_CTL_RESET_PDSP	        0
#define PA_REG_VAL_PDSP_CTL_STATE               (1 << 15)
#define PA_REG_VAL_PDSP_CTL_ENABLE              (1 << 1)
#define PA_REG_VAL_PDSP_CTL_SOFT_RESET          (1 << 0)
#define PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(pcval)	(((pcval) << 16)	\
						 | PA_REG_VAL_PDSP_CTL_ENABLE \
						 | PA_REG_VAL_PDSP_CTL_SOFT_RESET)

/* Number of mailbox slots for each PDPS */
#define PA_NUM_MAILBOX_SLOTS			4

static const unsigned int pdsp_code[] =  {
	0x2eff9196,
	0x85002096,
	0x0101f6f6,
	0x81002496,
	0xcf04fffe,
	0x2e808f86,
	0x240cecc2,
	0x2411e082,
	0x68e2ec05,
	0x59108926,
	0x24002104,
	0x2f000384,
	0x21000200,
	0x0101f7f7,
	0x81042497,
	0x24000c04,
	0x2f000384,
	0x2e808f86,
	0x24000004,
	0x240020c4,
	0x2f000384,
	0x2e808f8e,
	0x68e6fb04,
	0x68c7dc03,
	0x0101f8f8,
	0x21002400,
	0x68e6fd04,
	0x68c7de03,
	0x0101f9f9,
	0x21002400,
	0x0101fafa,
	0x810c249a,
	0x24002104,
	0x2f000384,
	0x8700e286,
	0x21000200,
	0x00f9f8e1,
	0x81082481,
	0x24002004,
	0x24000644,
	0x24000064,
	0x109e9ec5,
	0x2400b024,
	0x24000005,
	0x2f000384,
	0x8700e186,
	0x21000200,
	0x24000c04,
	0x2f000384,
	0x2e80878e,
	0x10eeeefb,
	0x10efeffc,
	0x10f0f0fd,
	0x10f1f1fe,
	0x24002104,
	0x2f000384,
	0x21000200
};

#endif /* __MACH_C6X_KEYSTONE_PA_H */

