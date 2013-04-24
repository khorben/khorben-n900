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

/*
 * Texas Instruments WiLink driver:
 * - WL1251
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/device.h>

#include <dev/spi/spivar.h>

/* variables */
struct wilink_softc {
	device_t		sc_dev;
	spi_tag_t		sc_spi;
	spi_addr_t		sc_addr;

	void *			sc_intr;
};

static int	wilink_match(device_t, cfdata_t, void *);
static void	wilink_attach(device_t, device_t, void *);
static int	wilink_detach(device_t, int);

static bool	wilink_suspend(device_t, const pmf_qual_t *);
static bool	wilink_resume(device_t, const pmf_qual_t *);

static int	wilink_power(struct wilink_softc *, bool);
static int	wilink_reset(struct wilink_softc *);

static int	wilink_intr(void *);

static int	wilink_read_1(struct wilink_softc *, uint8_t, uint8_t *);
static int	wilink_write_1(struct wilink_softc *, uint8_t, uint8_t);

CFATTACH_DECL_NEW(wilink, sizeof(struct wilink_softc),
	wilink_match, wilink_attach, wilink_detach, NULL);

static int
wilink_match(device_t parent, cfdata_t match, void *aux)
{
	return 1;
}

static void
wilink_attach(device_t parent, device_t self, void *aux)
{
	struct wilink_softc *sc = device_private(self);
	struct spi_attach_args *ia = aux;

	sc->sc_dev = self;
	sc->sc_spi = ia->ia_tag;
	sc->sc_addr = ia->ia_addr;

	wilink_reset(sc);

	sc->sc_intr = intr_establish(ia->ia_intr, IST_EDGE, IPL_VM, wilink_intr,
			sc);
	if (sc->sc_intr == NULL) {
		aprint_error("could not establish interrupt\n");
		return;
	}

	wilink_power(sc, true);

	aprint_normal(": WiLink 1251 wireless network interface\n");
	aprint_naive(": WiLink 1251 wireless network interface\n");

	spi_acquire_bus(sc->sc_spi, 0);

	spi_release_bus(sc->sc_spi, 0);

	if (!pmf_device_register(sc->sc_dev, wilink_suspend, wilink_resume)) {
		aprint_error_dev(sc->sc_dev,
		    "could not establish power handler\n");
	}
}

static int
wilink_detach(device_t self, int flags)
{
	struct wilink_softc *sc = device_private(self);

	wilink_power(sc, false);

	if (sc->sc_intr) {
		intr_disestablish(sc->sc_intr);
	}

	pmf_device_deregister(self);

	return 0;
}

static bool
wilink_suspend(device_t self, const pmf_qual_t *qual)
{
	struct wilink_softc *sc = device_private(self);

	if (sc->sc_state != false)
		wilink_power(sc, false);

	return true;
}

static bool
wilink_resume(device_t self, const pmf_qual_t *qual)
{
	struct wilink_softc *sc = device_private(self);

	if (sc->sc_state != false)
		wilink_power(sc, true);

	return true;
}

static int
wilink_power(struct wilink_softc *sc, bool on)
{
	spi_acquire_bus(sc->sc_spi, 0);

	/* FIXME implement */

	spi_release_bus(sc->sc_spi, 0);

	return 0;
}

static int
wilink_reset(struct wilink_softc *sc)
{
	uint8_t u8;

	spi_acquire_bus(sc->sc_spi, 0);

	/* FIXME implement */

	spi_release_bus(sc->sc_spi, 0);

	return 0;
}

static int
wilink_intr(void *v)
{
	spi_acquire_bus(sc->sc_spi, 0);

	/* FIXME implement */

	spi_release_bus(sc->sc_spi, 0);

	return 0;
}

static int
wilink_read_1(struct wilink_softc *sc, uint8_t reg, uint8_t *val)
{
	return spi_exec(sc->sc_spi, SPI_OP_READ_WITH_STOP, sc->sc_addr,
			&reg, sizeof(reg), val, sizeof(*val), 0);
}

static int
wilink_write_1(struct wilink_softc *sc, uint8_t reg, uint8_t val)
{
	uint8_t data[2] = { reg, val };

	return spi_exec(sc->sc_spi, SPI_OP_WRITE_WITH_STOP, sc->sc_addr,
			NULL, 0, data, sizeof(data), 0);
}
