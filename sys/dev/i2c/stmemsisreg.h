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

#ifndef _DEV_I2C_STMEMSISREG_H_
#define _DEV_I2C_STMEMSISREG_H_

/*
 * ST MEMS inertial sensors (accelerometers)
 * - LIS3LV02DL
 * - LIS302DL
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/device.h>

#include <dev/i2c/i2cvar.h>
#include <dev/sysmon/sysmonvar.h>

/* registers */
#define STMEMSIS_REG_WHO_AM_I		0x0f

#define STMEMSIS_REG_CTRL1		0x20
#define  STMEMSIS_CTRL1_XEN		__BIT(0)
#define  STMEMSIS_CTRL1_YEN		__BIT(1)
#define  STMEMSIS_CTRL1_ZEN		__BIT(2)
#define  STMEMSIS_CTRL1_STM		__BIT(3)
#define  STMEMSIS_CTRL1_STP		__BIT(4)
#define  STMEMSIS_CTRL1_FS		__BIT(5)
#define  STMEMSIS_CTRL1_PD		__BIT(6)
#define  STMEMSIS_CTRL1_DR		__BIT(7)

#define STMEMSIS_REG_CTRL2		0x21
#define  STMEMSIS_CTRL2_HP_COEFF1	__BIT(0)
#define  STMEMSIS_CTRL2_HP_COEFF2	__BIT(1)
#define  STMEMSIS_CTRL2_HP_FF_WU1	__BIT(2)
#define  STMEMSIS_CTRL2_HP_FF_WU2	__BIT(3)
#define  STMEMSIS_CTRL2_FDS		__BIT(4)
#define  STMEMSIS_CTRL2_BOOT		__BIT(6)
#define  STMEMSIS_CTRL2_SIM		__BIT(7)

#define STMEMSIS_REG_CTRL3		0x22
#define STMEMSIS_REG_FILTER_RESET	0x23

#define STMEMSIS_REG_STATUS		0x27
#define  STMEMSIS_STATUS_XDA		__BIT(0)
#define  STMEMSIS_STATUS_YDA		__BIT(1)
#define  STMEMSIS_STATUS_ZDA		__BIT(2)
#define  STMEMSIS_STATUS_ZYXDA		__BIT(3)
#define  STMEMSIS_STATUS_XOR		__BIT(4)
#define  STMEMSIS_STATUS_YOR		__BIT(5)
#define  STMEMSIS_STATUS_ZOR		__BIT(6)
#define  STMEMSIS_STATUS_ZXYOR		__BIT(7)

#define STMEMSIS_REG_OUTX		0x29
#define STMEMSIS_REG_OUTY		0x2b
#define STMEMSIS_REG_OUTZ		0x2d

#define STMEMSIS_REG_FF_WU_CFG1		0x30
#define  STMEMSIS_FF_WU_CFG1_XLIE	__BIT(0)
#define  STMEMSIS_FF_WU_CFG1_XHIE	__BIT(1)
#define  STMEMSIS_FF_WU_CFG1_YLIE	__BIT(2)
#define  STMEMSIS_FF_WU_CFG1_YHIE	__BIT(3)
#define  STMEMSIS_FF_WU_CFG1_ZLIE	__BIT(4)
#define  STMEMSIS_FF_WU_CFG1_ZHIE	__BIT(5)
#define  STMEMSIS_FF_WU_CFG1_LIR	__BIT(6)
#define  STMEMSIS_FF_WU_CFG1_AOI	__BIT(7)

#define STMEMSIS_REG_FF_WU_SRC1		0x31
#define STMEMSIS_REG_FF_WU_THS1		0x32
#define STMEMSIS_REG_FF_WU_DURATION_1	0x33
#define STMEMSIS_REG_FF_WU_CFG2		0x34
#define  STMEMSIS_FF_WU_CFG2_XLIE	__BIT(0)
#define  STMEMSIS_FF_WU_CFG2_XHIE	__BIT(1)
#define  STMEMSIS_FF_WU_CFG2_YLIE	__BIT(2)
#define  STMEMSIS_FF_WU_CFG2_YHIE	__BIT(3)
#define  STMEMSIS_FF_WU_CFG2_ZLIE	__BIT(4)
#define  STMEMSIS_FF_WU_CFG2_ZHIE	__BIT(5)
#define  STMEMSIS_FF_WU_CFG2_LIR	__BIT(6)
#define  STMEMSIS_FF_WU_CFG2_AOI	__BIT(7)

#define STMEMSIS_REG_FF_WU_SRC2		0x35
#define STMEMSIS_REG_FF_WU_THS2		0x36

#define STMEMSIS_REG_CLICK_CFG		0x38
#define  STMEMSIS_CLICK_CFG_X		__BIT(0)
#define  STMEMSIS_CLICK_CFG_X2		__BIT(1)
#define  STMEMSIS_CLICK_CFG_Y		__BIT(2)
#define  STMEMSIS_CLICK_CFG_Y2		__BIT(3)
#define  STMEMSIS_CLICK_CFG_Z		__BIT(4)
#define  STMEMSIS_CLICK_CFG_Z2		__BIT(5)
#define  STMEMSIS_CLICK_CFG_LIR		__BIT(6)

#define STMEMSIS_REG_CLICK_SRC		0x39
#define STMEMSIS_REG_CLICK_THSY_X	0x3b
#define STMEMSIS_REG_CLICK_THSZ		0x3c
#define STMEMSIS_REG_CLICK_TIMELIMIT	0x3d
#define STMEMSIS_REG_CLICK_LATENCY	0x3e
#define STMEMSIS_REG_CLICK_WINDOW	0x3f

#define STMEMSIS_SRC1_XL		__BIT(0)
#define STMEMSIS_SRC1_XE		__BIT(1)
#define STMEMSIS_SRC1_YL		__BIT(2)
#define STMEMSIS_SRC1_YH		__BIT(3)
#define STMEMSIS_SRC1_ZL		__BIT(4)
#define STMEMSIS_SRC1_ZH		__BIT(5)
#define STMEMSIS_SRC1_IA		__BIT(6)

#endif  /* _DEV_I2C_STMEMSISREG_H_ */
