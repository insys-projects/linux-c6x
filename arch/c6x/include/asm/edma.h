/*
 *  linux/arch/c6x/include/asm-c6x/edma.h
 *
 *  TI DAVINCI and TMS320C6x SoC EDMA definitions
 *
 *  Copyright (C) 2006, 2010, 2011 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
/******************************************************************************
 * DMA driver for DaVinci
 * DMA driver for Davinci abstractes each ParamEntry as a Logical DMA channel
 * for the user.So on Davinci the user can request 128 DAM channels
 *
 * Actual Physical DMA channels = 64 EDMA channels + 8 QDMA channels
 *
 * On davinci user can request for two kinds of Logical DMA channels
 * DMA MasterChannel -> ParamEntry which is associated with a DMA channel.
 *                      On Davinci there are (64 + 8) MasterChanneles
 *                      MasterChannel can be triggered by an event or manually
 *
 * DMA SlaveChannel  -> ParamEntry which is not associated with DMA cahnnel but
 *                      which can be used to associate with MasterChannel.
 *                      On Davinci there are (128-(64 + 8)) SlaveChannels
 *                      SlaveChannel can only be triggered by a MasterChannel
 *
 ******************************************************************************
 *
 * 2009-03-30   Aurelien Jacquiot / Nicolas Videau - Modified to support all
 *              TMS320C6x - Copyright (C) 2006, 2009, Texas Instruments Incorporated
 *
 ******************************************************************************
 */
#ifndef __ASM_ARCH_EDMA_H_
#define __ASM_ARCH_EDMA_H_

#include <mach/edma.h>

#endif /* __ASM_ARCH_EDMA_H_ */
