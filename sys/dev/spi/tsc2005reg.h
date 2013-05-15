/* $NetBSD$ */

/*
 * Copyright (c) 2013 Pierre Pronchery <khorben@defora.org>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef	_DEV_SPI_TSC2005REG_H_
#define	_DEV_SPI_TSC2005REG_H_

/*
 * Texas Instruments Touch Screen Controller:
 * - 2005
 */

#define TSC2005_CTL		0x80
#define  TSC2005_CTL_RM		__BIT(2)
#define  TSC2005_CTL_SWRST	__BIT(1)
#define  TSC2005_CTL_STS	__BIT(0)

#define TSC2005_CMD		0x00
#define  TSC2005_CMD_READ_REG	0x01
#define  TSC2005_CMD_WRITE_REG	0x00
#define  TSC2005_CMD_ADDR_SHIFT	3

#define TSC2005_REG_X		0x00
#define TSC2005_REG_Y		0x01
#define TSC2005_REG_Z1		0x02
#define TSC2005_REG_Z2		0x03
#define TSC2005_REG_AUX		0x04
#define TSC2005_REG_TEMP1	0x05
#define TSC2005_REG_TEMP2	0x06
#define TSC2005_REG_STATUS	0x07
#define TSC2005_REG_AUX_HIGH	0x08
#define TSC2005_REG_AUX_LOW	0x09
#define TSC2005_REG_TEMP_HIGH	0x0a
#define TSC2005_REG_TEMP_LOW	0x0b
#define TSC2005_REG_CFR0	0x0c
#define TSC2005_REG_CFR1	0x0d
#define TSC2005_REG_CFR2	0x0e
#define TSC2005_REG_CFSS	0x0f

#endif	/* _DEV_SPI_SPIVAR_H_ */
