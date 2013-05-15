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
 * Texas Instruments Touch Screen Controller:
 * - 2005
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/device.h>
#include <sys/workqueue.h>

#include <dev/spi/spivar.h>
#include <dev/sysmon/sysmonvar.h>
#include <dev/wscons/wsconsio.h>
#include <dev/wscons/wsmousevar.h>

#include <dev/spi/tsc2005reg.h>

enum tsc2005_sensors {
	TSC2005_SENSOR_TEMP1 = 0,
	TSC2005_SENSOR_TEMP2
};
#define TSC2005_SENSOR_LAST	TSC2005_SENSOR_TEMP2
#define TSC2005_SENSOR_COUNT	(TSC2005_SENSOR_LAST + 1)

#if 1
static const char const * tsc2005_sensors_names[TSC2005_SENSOR_COUNT] = {
	"temp1", "temp2"
};
#endif

/* variables */
struct tsc2005_softc {
	device_t		sc_dev;
	struct spi_handle	*sc_spi;

	void *			sc_intr;

	struct workqueue	*sc_workq;
	struct work		sc_work;
	bool			sc_queued;

	device_t		sc_wsmousedev;
	bool			sc_wsenabled;

	struct sysmon_envsys	*sc_sme;
	bool			sc_state;
	envsys_data_t		sc_sensor[TSC2005_SENSOR_COUNT];
};

static int	tsc2005_match(device_t, cfdata_t, void *);
static void	tsc2005_attach(device_t, device_t, void *);
static int	tsc2005_detach(device_t, int);

static int	tsc2005_reset(struct tsc2005_softc *);

static int	tsc2005_intr(void *);

static void	tsc2005_intr_work(struct work *, void *);

static int	tsc2005_read_reg(struct tsc2005_softc *, int, uint16_t *);
#if 0
static int	tsc2005_write_reg(struct tsc2005_softc *, int, uint16_t);
#endif

static int	tsc2005_enable(void *);
static void	tsc2005_disable(void *);
static int	tsc2005_ioctl(void *, u_long, void *, int, struct lwp *);

static const struct wsmouse_accessops tsc2005_accessops = {
	tsc2005_enable,
	tsc2005_ioctl,
	tsc2005_disable
};

CFATTACH_DECL_NEW(tsc2005ts, sizeof(struct tsc2005_softc),
		tsc2005_match, tsc2005_attach, tsc2005_detach, NULL);

static int
tsc2005_match(device_t parent, cfdata_t match, void *aux)
{
	struct spi_attach_args *sa = aux;

	spi_configure(sa->sa_handle, SPI_MODE_0, 6000000);
	return 1;
}

static void
tsc2005_attach(device_t parent, device_t self, void *aux)
{
	struct tsc2005_softc *sc = device_private(self);
	struct spi_attach_args *sa = aux;
#if 1
	struct wsmousedev_attach_args a;
	int i;
#endif
	int error;

	sc->sc_dev = self;
	sc->sc_spi = sa->sa_handle;

	aprint_normal(": TSC2005 touchscreen\n");
	aprint_naive(": TSC2005 touchscreen\n");

#if 0
	tsc2005_reset(sc);
#endif

#if 1
	sc->sc_intr = intr_establish(sa->sa_intr, IPL_BIO, IST_LEVEL_LOW,
			tsc2005_intr, sc);
#else
	sc->sc_intr = intr_establish(sa->sa_intr, IPL_BIO, IST_EDGE_FALLING,
			tsc2005_intr, sc);
#endif
	if (sc->sc_intr == NULL) {
		aprint_error_dev(sc->sc_dev, "couldn't establish interrupt\n");
	}

	error = workqueue_create(&sc->sc_workq, device_xname(sc->sc_dev),
			tsc2005_intr_work, sc, PRIO_MAX, IPL_BIO, 0);
	if (error) {
		aprint_error_dev(sc->sc_dev, "couldn't create workqueue\n");
	}
	sc->sc_queued = false;

#if 1
	/* attach wsmouse */
	a.accessops = &tsc2005_accessops;
	a.accesscookie = sc;
	sc->sc_wsmousedev = config_found(self, &a, wsmousedevprint);
	sc->sc_wsenabled = false;
#endif

#if 1
	/* initialize the sensors */
	sc->sc_sme = sysmon_envsys_create();
	for (i = 0; i < TSC2005_SENSOR_COUNT; i++) {
		sc->sc_sensor[i].units = ENVSYS_INTEGER;
		strlcpy(sc->sc_sensor[i].desc, tsc2005_sensors_names[i],
				sizeof(sc->sc_sensor[i].desc));
		sc->sc_sensor[i].state = ENVSYS_SVALID;
		sc->sc_sensor[i].flags = ENVSYS_FHAS_ENTROPY;
		if (sysmon_envsys_sensor_attach(sc->sc_sme,
					&sc->sc_sensor[i])) {
			aprint_error_dev(sc->sc_dev,
					"couldn't attach sensor %s\n",
					sc->sc_sensor[i].desc);
			sysmon_envsys_destroy(sc->sc_sme);
			sc->sc_sme = NULL;
			break;
		}
	}

	/* register the sensors */
	if (sc->sc_sme) {
		sc->sc_sme->sme_name = device_xname(self);
		sc->sc_sme->sme_flags = SME_DISABLE_REFRESH;

		error = sysmon_envsys_register(sc->sc_sme);
		if (error) {
			aprint_error_dev(self,
					"unable to register with sysmon (%d)\n",
					error);
			sysmon_envsys_destroy(sc->sc_sme);
			sc->sc_sme = NULL;
		}
	}
#endif

	if (!pmf_device_register(sc->sc_dev, NULL, NULL)) {
		aprint_error_dev(sc->sc_dev,
				"couldn't establish power handler\n");
	}
}

static int
tsc2005_detach(device_t self, int flags)
{
	struct tsc2005_softc *sc = device_private(self);

	if (sc->sc_sme) {
		sysmon_envsys_unregister(sc->sc_sme);
	}

	if (sc->sc_workq) {
		workqueue_destroy(sc->sc_workq);
	}

	if (sc->sc_intr) {
		intr_disestablish(sc->sc_intr);
	}

	pmf_device_deregister(self);

	return 0;
}

static int
tsc2005_reset(struct tsc2005_softc *sc)
{
	uint8_t u8;

	/* send a Control Byte 1 with the SWRST bit set */
	u8 = TSC2005_CTL | TSC2005_CTL_SWRST;
	spi_send(sc->sc_spi, sizeof(u8), &u8);

	/* send a Control Byte 1 with the SWRST bit unset */
	u8 = TSC2005_CTL;
	spi_send(sc->sc_spi, sizeof(u8), &u8);
	return 0;
}

static int
tsc2005_intr(void *v)
{
	struct tsc2005_softc *sc = v;

	if (!sc->sc_queued) {
		workqueue_enqueue(sc->sc_workq, &sc->sc_work, NULL);
		sc->sc_queued = true;
	}

	return 0;
}

static void
tsc2005_intr_work(struct work *wk, void *arg)
{
	struct tsc2005_softc *sc = arg;
	uint16_t status;
#if 1
	uint16_t x;
	uint16_t y;
	uint16_t z;
	int buttons = 0;
	int flags = WSMOUSE_INPUT_ABSOLUTE_X | WSMOUSE_INPUT_ABSOLUTE_Y;
	int s;
#endif
#if 1
	static int reset = 0;

	if (reset == 0) {
		tsc2005_reset(sc);
		reset = 1;
		sc->sc_queued = false;
		intr_enable(sc->sc_intr);
		return;
	}
#endif

	tsc2005_read_reg(sc, TSC2005_REG_STATUS, &status);

	if (!sc->sc_wsenabled) {
		sc->sc_queued = false;
		intr_enable(sc->sc_intr);
		return;
	}

#if 1
	tsc2005_read_reg(sc, TSC2005_REG_X, &x);
	tsc2005_read_reg(sc, TSC2005_REG_Y, &y);
	tsc2005_read_reg(sc, TSC2005_REG_Z1, &z);

	if (sc->sc_wsmousedev != NULL) {
		s = spltty();
		wsmouse_input(sc->sc_wsmousedev, buttons, x, y, z, 0, flags);
		splx(s);
	}
#endif

	sc->sc_queued = false;
	intr_enable(sc->sc_intr);
}

static int
tsc2005_read_reg(struct tsc2005_softc *sc, int reg, uint16_t *data)
{
	uint8_t buf[2];

	buf[0] = TSC2005_CMD | TSC2005_CMD_READ_REG
		| (reg << TSC2005_CMD_ADDR_SHIFT);
	spi_send_recv(sc->sc_spi, sizeof(buf[0]), &buf[0], sizeof(buf), buf);
	*data = (buf[0] << 8) | buf[1];
	return 0;
}

#if 0
static int
tsc2005_write_reg(struct tsc2005_softc *sc, int reg, uint16_t data)
{
	uint8_t buf[3];

	buf[0] = TSC2005_CMD | TSC2005_CMD_WRITE_REG
		| (reg << TSC2005_CMD_ADDR_SHIFT);
	buf[1] = (data >> 8) & 0xff;
	buf[2] = data & 0xff;
	return spi_send(sc->sc_spi, sizeof(buf), buf);
}
#endif

static int
tsc2005_enable(void *v)
{
	struct tsc2005_softc *sc = v;

	sc->sc_wsenabled = true;
	return 0;
}

static void
tsc2005_disable(void *v)
{
	struct tsc2005_softc *sc = v;

	sc->sc_wsenabled = false;
}

static int
tsc2005_ioctl(void *v, u_long cmd, void *data, int flag, struct lwp *l)
{
	switch (cmd) {
		case WSMOUSEIO_GTYPE:
			*(u_int *)data = WSMOUSE_TYPE_TPANEL;
			return 0;
	}

	return EPASSTHROUGH;
}
