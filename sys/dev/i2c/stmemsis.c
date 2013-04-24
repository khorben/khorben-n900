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

#include <dev/i2c/stmemsisreg.h>

/* constants */
#define STMEMSIS_ID_3LV02DL		0x3a /* LIS3LV02DL */
#define STMEMSIS_ID_302DL		0x3b /* LIS302DL   */

enum stmemsis_sensors {
	STMEMSIS_SENSOR_XACCEL = 0,
	STMEMSIS_SENSOR_YACCEL,
	STMEMSIS_SENSOR_ZACCEL
};
#define STMEMSIS_SENSOR_LAST		STMEMSIS_SENSOR_ZACCEL
#define STMEMSIS_SENSOR_CNT		(STMEMSIS_SENSOR_LAST + 1)

/* variables */
struct stmemsis_softc {
	device_t		sc_dev;
	i2c_tag_t		sc_i2c;
	i2c_addr_t		sc_addr;
	uint8_t			sc_id;
	void *			sc_intr1;
	void *			sc_intr2;

	struct sysmon_envsys	*sc_sme;
	bool			sc_state;
	envsys_data_t		sc_sensor[STMEMSIS_SENSOR_CNT];
};

static int	stmemsis_match(device_t, cfdata_t, void *);
static void	stmemsis_attach(device_t, device_t, void *);
static int	stmemsis_detach(device_t, int);

static bool	stmemsis_suspend(device_t, const pmf_qual_t *);
static bool	stmemsis_resume(device_t, const pmf_qual_t *);

static int	stmemsis_power(struct stmemsis_softc *, bool);
static int	stmemsis_reset(struct stmemsis_softc *);

static int	stmemsis_intr(void *);

static int	stmemsis_read_1(struct stmemsis_softc *, uint8_t, uint8_t *);
static int	stmemsis_write_1(struct stmemsis_softc *, uint8_t, uint8_t);

CFATTACH_DECL_NEW(stmemsis, sizeof(struct stmemsis_softc),
	stmemsis_match, stmemsis_attach, stmemsis_detach, NULL);

static int
stmemsis_match(device_t parent, cfdata_t match, void *aux)
{
	struct i2c_attach_args *ia = aux;
	uint8_t reg;
	uint8_t val;

	reg = STMEMSIS_REG_WHO_AM_I;
	if (iic_exec(ia->ia_tag, I2C_OP_READ_WITH_STOP, ia->ia_addr,
				&reg, sizeof(reg), &val, sizeof(val), 0) != 0)
		return 0;
	switch(val)
	{
		case STMEMSIS_ID_302DL:
#if 0 /* FIXME there are differences to address */
		case STMEMSIS_ID_3LV02DL:
#endif
			return 1;
		default:
			return 0;
	}
}

static void
stmemsis_attach(device_t parent, device_t self, void *aux)
{
	struct stmemsis_softc *sc = device_private(self);
	struct i2c_attach_args *ia = aux;
	uint8_t u8;
	int i;
	int error;

	sc->sc_dev = self;
	sc->sc_i2c = ia->ia_tag;
	sc->sc_addr = ia->ia_addr;
	sc->sc_id = 0;
	sc->sc_state = false;

	stmemsis_reset(sc);

	sc->sc_intr1 = intr_establish(ia->ia_intr, IPL_VM, IST_LEVEL,
			stmemsis_intr, sc);
	if (sc->sc_intr1 == NULL) {
		aprint_error("couldn't establish interrupt\n");
		return;
	}

	sc->sc_intr2 = intr_establish(ia->ia_intr + 1, IPL_VM, IST_LEVEL,
			stmemsis_intr, sc);
	if (sc->sc_intr2 == NULL) {
		aprint_error("couldn't establish interrupt\n");
		intr_disestablish(sc->sc_intr1);
		return;
	}

	stmemsis_power(sc, true);
	sc->sc_state = true;

	aprint_normal(": motion sensor\n");
	aprint_naive(": motion sensor\n");

	iic_acquire_bus(sc->sc_i2c, 0);

	/* keep the device ID */
	stmemsis_read_1(sc, STMEMSIS_REG_WHO_AM_I, &sc->sc_id);

	/* enable free-fall wake up */
	u8 = STMEMSIS_FF_WU_CFG1_XLIE | STMEMSIS_FF_WU_CFG1_XHIE
		| STMEMSIS_FF_WU_CFG1_YLIE | STMEMSIS_FF_WU_CFG1_YHIE
		| STMEMSIS_FF_WU_CFG1_ZLIE | STMEMSIS_FF_WU_CFG1_ZHIE;
	stmemsis_write_1(sc, STMEMSIS_REG_FF_WU_CFG1, u8);
	u8 = STMEMSIS_FF_WU_CFG2_XLIE | STMEMSIS_FF_WU_CFG2_XHIE
		| STMEMSIS_FF_WU_CFG2_YLIE | STMEMSIS_FF_WU_CFG2_YHIE
		| STMEMSIS_FF_WU_CFG2_ZLIE | STMEMSIS_FF_WU_CFG2_ZHIE;
	stmemsis_write_1(sc, STMEMSIS_REG_FF_WU_CFG2, u8);

	/* enable clicks */
	u8 = STMEMSIS_CLICK_CFG_X | STMEMSIS_CLICK_CFG_X2
		| STMEMSIS_CLICK_CFG_Y | STMEMSIS_CLICK_CFG_Y2
		| STMEMSIS_CLICK_CFG_Z | STMEMSIS_CLICK_CFG_Z2;
	stmemsis_write_1(sc, STMEMSIS_REG_CLICK_CFG, u8);

	iic_release_bus(sc->sc_i2c, 0);

	/* initialize the sensors */
#define INITDATA(idx, unit, string)					\
	sc->sc_sensor[idx].units = unit;				\
	strlcpy(sc->sc_sensor[idx].desc, string,			\
			sizeof(sc->sc_sensor[idx].desc));
	INITDATA(STMEMSIS_SENSOR_XACCEL, ENVSYS_INTEGER, "x-acceleration");
	INITDATA(STMEMSIS_SENSOR_YACCEL, ENVSYS_INTEGER, "y-acceleration");
	INITDATA(STMEMSIS_SENSOR_ZACCEL, ENVSYS_INTEGER, "z-acceleration");

	sc->sc_sme = sysmon_envsys_create();
	for (i = 0; i < STMEMSIS_SENSOR_CNT; i++) {
		sc->sc_sensor[i].state = ENVSYS_SVALID;

		if (sc->sc_sensor[i].units == ENVSYS_INTEGER) {
			sc->sc_sensor[i].flags = ENVSYS_FHAS_ENTROPY;
		}

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

	/* register with sysmon_envsys(9) */
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

	if (!pmf_device_register(sc->sc_dev, stmemsis_suspend,
				stmemsis_resume)) {
		aprint_error_dev(sc->sc_dev,
		    "could not establish power handler\n");
	}
}

static int
stmemsis_detach(device_t self, int flags)
{
	struct stmemsis_softc *sc = device_private(self);

	stmemsis_power(sc, false);

	if (sc->sc_intr1) {
		intr_disestablish(sc->sc_intr1);
	}
	if (sc->sc_intr2) {
		intr_disestablish(sc->sc_intr2);
	}

	if (sc->sc_sme) {
		sysmon_envsys_unregister(sc->sc_sme);
	}

	pmf_device_deregister(self);

	return 0;
}

static bool
stmemsis_suspend(device_t self, const pmf_qual_t *qual)
{
	struct stmemsis_softc *sc = device_private(self);

	if (sc->sc_state != false)
		stmemsis_power(sc, false);

	return true;
}

static bool
stmemsis_resume(device_t self, const pmf_qual_t *qual)
{
	struct stmemsis_softc *sc = device_private(self);

	if (sc->sc_state != false)
		stmemsis_power(sc, true);

	return true;
}

static int
stmemsis_power(struct stmemsis_softc *sc, bool on)
{
	uint8_t u8;

	iic_acquire_bus(sc->sc_i2c, 0);

	stmemsis_read_1(sc, STMEMSIS_REG_CTRL1, &u8);
	if (on)
		u8 |= STMEMSIS_CTRL1_PD;
	else
		u8 &= ~(STMEMSIS_CTRL1_PD);
	stmemsis_write_1(sc, STMEMSIS_REG_CTRL1, u8);

	iic_release_bus(sc->sc_i2c, 0);

	return 0;
}

static int
stmemsis_reset(struct stmemsis_softc *sc)
{
	uint8_t u8;
	int retry;

	iic_acquire_bus(sc->sc_i2c, 0);

	/* reboot the device */
	if (stmemsis_read_1(sc, STMEMSIS_REG_CTRL2, &u8) == 0) {
		u8 |= STMEMSIS_CTRL2_BOOT;
		stmemsis_write_1(sc, STMEMSIS_REG_CTRL2, u8);

		retry = 50;
		while (--retry) {
			stmemsis_read_1(sc, STMEMSIS_REG_CTRL2, &u8);
			if ((u8 & STMEMSIS_CTRL2_BOOT) == 0)
				break;
			delay(1000);
		}
		if (retry == 0)
			aprint_error_dev(sc->sc_dev, "reset timeout\n");
	}

	/* reset the internal filter */
	stmemsis_read_1(sc, STMEMSIS_REG_FILTER_RESET, &u8);

	/* enable all three axis */
	u8 = STMEMSIS_CTRL1_XEN | STMEMSIS_CTRL1_YEN | STMEMSIS_CTRL1_ZEN;
	stmemsis_write_1(sc, STMEMSIS_REG_CTRL1, u8);

#if 0
	/* reset the free-fall thresholds */
	stmemsis_write_1(sc, STMEMSIS_REG_FF_WU_THS1, 0);
	stmemsis_write_1(sc, STMEMSIS_REG_FF_WU_THS2, 0);
#endif

	/* reset the click thresholds */
	stmemsis_write_1(sc, STMEMSIS_REG_CLICK_THSY_X, (0x8 << 4) | 0x8);
	stmemsis_write_1(sc, STMEMSIS_REG_CLICK_THSZ, 0xa);
	stmemsis_write_1(sc, STMEMSIS_REG_CLICK_TIMELIMIT, 0x9);
	stmemsis_write_1(sc, STMEMSIS_REG_CLICK_LATENCY, 50);

	iic_release_bus(sc->sc_i2c, 0);

	return 0;
}

static int
stmemsis_intr(void *v)
{
	struct stmemsis_softc *sc = v;
	uint8_t val;
	uint8_t u8;

	iic_acquire_bus(sc->sc_i2c, 0);

	if (stmemsis_read_1(sc, STMEMSIS_REG_STATUS, &val) == 0) {
		if(val & STMEMSIS_STATUS_XDA) {
			stmemsis_read_1(sc, STMEMSIS_REG_OUTX, &u8);
			sc->sc_sensor[STMEMSIS_SENSOR_XACCEL].value_cur = u8;
		}
		if(val & STMEMSIS_STATUS_YDA) {
			stmemsis_read_1(sc, STMEMSIS_REG_OUTY, &u8);
			sc->sc_sensor[STMEMSIS_SENSOR_YACCEL].value_cur = u8;
		}
		if(val & STMEMSIS_STATUS_ZDA) {
			stmemsis_read_1(sc, STMEMSIS_REG_OUTZ, &u8);
			sc->sc_sensor[STMEMSIS_SENSOR_ZACCEL].value_cur = u8;
		}
	}

#if 0
	aprint_normal_dev(sc->sc_dev, "%u %u %u\n",
			sc->sc_sensor[STMEMSIS_SENSOR_XACCEL].value_cur,
			sc->sc_sensor[STMEMSIS_SENSOR_YACCEL].value_cur,
			sc->sc_sensor[STMEMSIS_SENSOR_ZACCEL].value_cur);
#endif

	if (stmemsis_read_1(sc, STMEMSIS_REG_FF_WU_SRC1, &val) == 0) {
		/* FIXME implement */
	}

	if (stmemsis_read_1(sc, STMEMSIS_REG_FF_WU_SRC2, &val) == 0) {
		/* FIXME implement */
	}

	if (stmemsis_read_1(sc, STMEMSIS_REG_CLICK_SRC, &val) == 0) {
		/* FIXME implement */
	}

	iic_release_bus(sc->sc_i2c, 0);

	return 1;
}

static int
stmemsis_read_1(struct stmemsis_softc *sc, uint8_t reg, uint8_t *val)
{
	return iic_exec(sc->sc_i2c, I2C_OP_READ_WITH_STOP, sc->sc_addr,
			&reg, sizeof(reg), val, sizeof(*val), 0);
}

static int
stmemsis_write_1(struct stmemsis_softc *sc, uint8_t reg, uint8_t val)
{
	uint8_t data[2] = { reg, val };

	return iic_exec(sc->sc_i2c, I2C_OP_WRITE_WITH_STOP, sc->sc_addr,
			NULL, 0, data, sizeof(data), 0);
}
