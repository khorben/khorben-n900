/* $NetBSD$ */

/*
 * Texas Instruments LP5523 Programmable 9-Output LED Driver
 *
 * Copyright (c) 2013 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Pierre Pronchery (khorben@defora.org).
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

#ifndef _DEV_I2C_LP5523REG_H_
#define _DEV_I2C_LP5523REG_H_

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

/* registers */
#define LP5523_REG_ENGINE_CNTRL1	0x00
#define LP5523_REG_ENGINE_CNTRL2	0x01
#define LP5523_REG_OUTPUT_DIRECT_MSB	0x02
#define LP5523_REG_OUTPUT_DIRECT_LSB	0x03
#define LP5523_REG_OUTPUT_ONOFF_MSB	0x04
#define LP5523_REG_OUTPUT_ONOFF_LSB	0x05
#define LP5523_REG_D1_CONTROL		0x06
#define LP5523_REG_D2_CONTROL		0x07
#define LP5523_REG_D3_CONTROL		0x08
#define LP5523_REG_D4_CONTROL		0x09
#define LP5523_REG_D5_CONTROL		0x0a
#define LP5523_REG_D6_CONTROL		0x0b
#define LP5523_REG_D7_CONTROL		0x0c
#define LP5523_REG_D8_CONTROL		0x0d
#define LP5523_REG_D9_CONTROL		0x0e
#define LP5523_REG_D1_PWM		0x16
#define LP5523_REG_D2_PWM		0x17
#define LP5523_REG_D3_PWM		0x18
#define LP5523_REG_D4_PWM		0x19
#define LP5523_REG_D5_PWM		0x1a
#define LP5523_REG_D6_PWM		0x1b
#define LP5523_REG_D7_PWM		0x1c
#define LP5523_REG_D8_PWM		0x1d
#define LP5523_REG_D9_PWM		0x1e
#define LP5523_REG_D1_CURRENT		0x26
#define LP5523_REG_D2_CURRENT		0x27
#define LP5523_REG_D3_CURRENT		0x28
#define LP5523_REG_D4_CURRENT		0x29
#define LP5523_REG_D5_CURRENT		0x2a
#define LP5523_REG_D6_CURRENT		0x2b
#define LP5523_REG_D7_CURRENT		0x2c
#define LP5523_REG_D8_CURRENT		0x2d
#define LP5523_REG_D9_CURRENT		0x2e
#define LP5523_REG_MISC			0x36
#define LP5523_REG_ENGINE1_PC		0x37
#define LP5523_REG_ENGINE2_PC		0x38
#define LP5523_REG_ENGINE3_PC		0x39

#endif  /* _DEV_I2C_LP5523REG_H_ */
