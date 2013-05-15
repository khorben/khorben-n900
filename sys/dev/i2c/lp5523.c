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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/device.h>
#include <sys/sysctl.h>

#include <dev/i2c/i2cvar.h>

#include <dev/i2c/lp5523reg.h>

/* constants */
enum lp5523_leds {
	LP5523_LED_0 = 0,
	LP5523_LED_1,
	LP5523_LED_2,
	LP5523_LED_3,
	LP5523_LED_4,
	LP5523_LED_5,
	LP5523_LED_6,
	LP5523_LED_7,
	LP5523_LED_8
};
#define LP5523_LED_LAST		LP5523_LED_8
#define LP5523_LED_CNT		(LP5523_LED_LAST + 1)

/* variables */
struct lp5523_softc {
	device_t		sc_dev;
	i2c_tag_t		sc_i2c;
	i2c_addr_t		sc_addr;

	struct sysctllog *sc_sysctllog;
};

static int	lp5523_match(device_t, cfdata_t, void *);
static void	lp5523_attach(device_t, device_t, void *);
static int	lp5523_detach(device_t, int);

static int	lp5523_reset(struct lp5523_softc *);

#if 0
static int	lp5523_read_1(struct lp5523_softc *, uint8_t, uint8_t *);
static int	lp5523_write_1(struct lp5523_softc *, uint8_t, uint8_t);
#endif

CFATTACH_DECL_NEW(lp5523led, sizeof(struct lp5523_softc),
	lp5523_match, lp5523_attach, lp5523_detach, NULL);

static int
lp5523_match(device_t parent, cfdata_t match, void *aux)
{
	return 1;
}

static void
lp5523_attach(device_t parent, device_t self, void *aux)
{
	struct lp5523_softc *sc = device_private(self);
	struct i2c_attach_args *ia = aux;

	sc->sc_dev = self;
	sc->sc_i2c = ia->ia_tag;
	sc->sc_addr = ia->ia_addr;

	lp5523_reset(sc);

	aprint_normal(": LED driver\n");
	aprint_naive(": LED driver\n");

	/* FIXME implement */

	if (!pmf_device_register(sc->sc_dev, NULL, NULL)) {
		aprint_error_dev(sc->sc_dev,
		    "could not establish power handler\n");
	}
}

static int
lp5523_detach(device_t self, int flags)
{
	pmf_device_deregister(self);

	return 0;
}

static int
lp5523_reset(struct lp5523_softc *sc)
{
	/* FIXME implement */
	return 0;
}

#if 0
static int
lp5523_read_1(struct lp5523_softc *sc, uint8_t reg, uint8_t *val)
{
	return iic_exec(sc->sc_i2c, I2C_OP_READ_WITH_STOP, sc->sc_addr,
			&reg, sizeof(reg), val, sizeof(*val), 0);
}

static int
lp5523_write_1(struct lp5523_softc *sc, uint8_t reg, uint8_t val)
{
	uint8_t data[2] = { reg, val };

	return iic_exec(sc->sc_i2c, I2C_OP_WRITE_WITH_STOP, sc->sc_addr,
			NULL, 0, data, sizeof(data), 0);
}
#endif
