/* $NetBSD: tps65950.c,v 1.3 2012/12/31 21:45:36 jmcneill Exp $ */

/*-
 * Copyright (c) 2012 Jared D. McNeill <jmcneill@invisible.ca>
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
 * TI TPS65950 OMAP Power Management and System Companion Device
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: tps65950.c,v 1.3 2012/12/31 21:45:36 jmcneill Exp $");

#define _INTR_PRIVATE

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/workqueue.h>
#include <sys/conf.h>
#include <sys/bus.h>
#include <sys/kmem.h>
#include <sys/sysctl.h>
#include <sys/wdog.h>

#include <dev/i2c/i2cvar.h>

#ifndef NGPIO
# define NGPIO 1 /* XXX */
#endif
#if NGPIO > 0
#include <arm/pic/picvar.h>
#include <sys/gpio.h>
#include <dev/gpio/gpiovar.h>
#endif /* NGPIO > 0 */

#if defined(OMAP_3430)
#include <dev/wscons/wsconsio.h>
#include <dev/wscons/wskbdvar.h>
#include <dev/wscons/wsksymdef.h>
#include <dev/wscons/wsksymvar.h>
#endif

#include <dev/clock_subr.h>
#include <dev/sysmon/sysmonvar.h>

/* Default watchdog period, in seconds */
#ifndef TPS65950_WDOG_DEFAULT_PERIOD
#define TPS65950_WDOG_DEFAULT_PERIOD	30
#endif

/* I2C Bus Addressing */
#define	TPS65950_ADDR_ID1		0x48	/* GP */
#define TPS65950_ADDR_ID2		0x49	/* GP */
#define TPS65950_ADDR_ID3		0x4a	/* GP */
#define TPS65950_ADDR_ID4		0x4b	/* GP */
#define TPS65950_ADDR_ID5		0x12	/* SmartReflex */

/* BCI */
#define TPS65950_BCI_BASE		0xb9
#define TPS65950_BCI_REG_BCIISR1A	(TPS65950_BCI_BASE + 0x00)
#define TPS65950_BCI_REG_BCIISR2A	(TPS65950_BCI_BASE + 0x01)
#define TPS65950_BCI_REG_BCIIMR1A	(TPS65950_BCI_BASE + 0x02)
#define TPS65950_BCI_REG_BCIIMR2A	(TPS65950_BCI_BASE + 0x03)

/* ID1: KEYPAD */
#define TPS65950_KEYPAD_BASE			0xd2
#define TPS65950_KEYPAD_REG_CTRL_REG		(TPS65950_KEYPAD_BASE + 0x00)
#define  TPS65950_KEYPAD_REG_CTRL_REG_KBD_ON	__BIT(6)
#define  TPS65950_KEYPAD_REG_CTRL_REG_RP_EN	__BIT(5)
#define  TPS65950_KEYPAD_REG_CTRL_REG_TOLE_EN	__BIT(4)
#define  TPS65950_KEYPAD_REG_CTRL_REG_TOE_EN	__BIT(3)
#define  TPS65950_KEYPAD_REG_CTRL_REG_LK_EN	__BIT(2)
#define  TPS65950_KEYPAD_REG_CTRL_REG_SOFTMODEN	__BIT(1)
#define  TPS65950_KEYPAD_REG_CTRL_REG_SOFT_NRST	__BIT(0)
#define TPS65950_KEYPAD_REG_FULL_CODE_7_0	(TPS65950_KEYPAD_BASE + 0x09)
#define TPS65950_KEYPAD_REG_FULL_CODE_15_8	(TPS65950_KEYPAD_BASE + 0x0a)
#define TPS65950_KEYPAD_REG_FULL_CODE_23_16	(TPS65950_KEYPAD_BASE + 0x0b)
#define TPS65950_KEYPAD_REG_FULL_CODE_31_24	(TPS65950_KEYPAD_BASE + 0x0c)
#define TPS65950_KEYPAD_REG_FULL_CODE_39_32	(TPS65950_KEYPAD_BASE + 0x0d)
#define TPS65950_KEYPAD_REG_FULL_CODE_47_40	(TPS65950_KEYPAD_BASE + 0x0e)
#define TPS65950_KEYPAD_REG_FULL_CODE_55_48	(TPS65950_KEYPAD_BASE + 0x0f)
#define TPS65950_KEYPAD_REG_FULL_CODE_63_56	(TPS65950_KEYPAD_BASE + 0x10)
#define TPS65950_KEYPAD_REG_ISR1		(TPS65950_KEYPAD_BASE + 0x11)
#define  TPS65950_KEYPAD_REG_ISR1_ITKPISR1	__BIT(0)
#define TPS65950_KEYPAD_REG_IMR1		(TPS65950_KEYPAD_BASE + 0x12)
#define  TPS65950_KEYPAD_REG_IMR1_ITMISIMR1	__BIT(3)
#define  TPS65950_KEYPAD_REG_IMR1_ITTOIMR1	__BIT(2)
#define  TPS65950_KEYPAD_REG_IMR1_ITLKIMR1	__BIT(1)
#define  TPS65950_KEYPAD_REG_IMR1_ITKPIMR1	__BIT(0)
#define TPS65950_KEYPAD_REG_SIH_CTRL		(TPS65950_KEYPAD_BASE + 0x17)
#define  TPS65950_KEYPAD_REG_SIH_CTRL_COR	__BIT(2)
#define  TPS65950_KEYPAD_REG_SIH_CTRL_PENDDIS	__BIT(1)
#define  TPS65950_KEYPAD_REG_SIH_CTRL_EXCLEN	__BIT(0)

/* ID2 */
#define TPS65950_ID2_IDCODE_7_0		0x85
#define TPS65950_ID2_IDCODE_15_8	0x86
#define TPS65950_ID2_IDCODE_23_16	0x87
#define TPS65950_ID2_IDCODE_31_24	0x88
#define TPS65950_ID2_UNLOCK_TEST_REG	0x97
#define TPS65950_ID2_UNLOCK_TEST_REG_MAGIC 0x49

/* ID2: PIH */
#define TPS65950_PIH_BASE		0x80
#define TPS65950_PIH_REG_ISR_P1		(TPS65950_PIH_BASE + 0x01)
#define  TPS65950_PIH_REG_ISR_P1_ISR7	__BIT(7)
#define  TPS65950_PIH_REG_ISR_P1_ISR6	__BIT(6)
#define  TPS65950_PIH_REG_ISR_P1_ISR5	__BIT(5)
#define  TPS65950_PIH_REG_ISR_P1_ISR4	__BIT(4)
#define  TPS65950_PIH_REG_ISR_P1_ISR3	__BIT(3)
#define  TPS65950_PIH_REG_ISR_P1_ISR2	__BIT(2)
#define  TPS65950_PIH_REG_ISR_P1_ISR1	__BIT(1)
#define  TPS65950_PIH_REG_ISR_P1_ISR0	__BIT(0)
#define TPS65950_PIH_REG_ISR_P2		(TPS65950_PIH_BASE + 0x02)
#define  TPS65950_PIH_REG_ISR_P2_ISR7	__BIT(7)
#define  TPS65950_PIH_REG_ISR_P2_ISR6	__BIT(6)
#define  TPS65950_PIH_REG_ISR_P2_ISR5	__BIT(5)
#define  TPS65950_PIH_REG_ISR_P2_ISR4	__BIT(4)
#define  TPS65950_PIH_REG_ISR_P2_ISR3	__BIT(3)
#define  TPS65950_PIH_REG_ISR_P2_ISR2	__BIT(2)
#define  TPS65950_PIH_REG_ISR_P2_ISR1	__BIT(1)
#define  TPS65950_PIH_REG_ISR_P2_ISR0	__BIT(0)
#define TPS65950_PIH_REG_SIR		(TPS65950_PIH_BASE + 0x03)

/* ID2: GPIO */
#define TPS65950_GPIO_BASE		0x98
#define TPS65950_GPIO_GPIODATAIN1	(TPS65950_GPIO_BASE + 0x01)
#define TPS65950_GPIO_GPIODATAIN2	(TPS65950_GPIO_BASE + 0x02)
#define TPS65950_GPIO_GPIODATAIN3	(TPS65950_GPIO_BASE + 0x03)
#define TPS65950_GPIO_GPIODATADIR1	(TPS65950_GPIO_BASE + 0x04)
#define TPS65950_GPIO_GPIODATADIR2	(TPS65950_GPIO_BASE + 0x05)
#define TPS65950_GPIO_GPIODATADIR3	(TPS65950_GPIO_BASE + 0x06)
#define TPS65950_GPIO_GPIODATAOUT1	(TPS65950_GPIO_BASE + 0x07)
#define TPS65950_GPIO_GPIODATAOUT2	(TPS65950_GPIO_BASE + 0x08)
#define TPS65950_GPIO_GPIODATAOUT3	(TPS65950_GPIO_BASE + 0x09)
#define TPS65950_GPIO_CLEARGPIODATAOUT1	(TPS65950_GPIO_BASE + 0x0a)
#define TPS65950_GPIO_CLEARGPIODATAOUT2	(TPS65950_GPIO_BASE + 0x0b)
#define TPS65950_GPIO_CLEARGPIODATAOUT3	(TPS65950_GPIO_BASE + 0x0c)
#define TPS65950_GPIO_SETGPIODATAOUT1	(TPS65950_GPIO_BASE + 0x0d)
#define TPS65950_GPIO_SETGPIODATAOUT2	(TPS65950_GPIO_BASE + 0x0e)
#define TPS65950_GPIO_SETGPIODATAOUT3	(TPS65950_GPIO_BASE + 0x0f)
#define TPS65950_GPIO_GPIO_DEBEN1	(TPS65950_GPIO_BASE + 0x10)
#define TPS65950_GPIO_GPIO_DEBEN2	(TPS65950_GPIO_BASE + 0x11)
#define TPS65950_GPIO_GPIO_DEBEN3	(TPS65950_GPIO_BASE + 0x12)
#define TPS65950_GPIO_GPIO_CTRL		(TPS65950_GPIO_BASE + 0x13)
#define TPS65950_GPIO_GPIOPUPDCTR1	(TPS65950_GPIO_BASE + 0x14)
#define TPS65950_GPIO_GPIOPUPDCTR2	(TPS65950_GPIO_BASE + 0x15)
#define TPS65950_GPIO_GPIOPUPDCTR3	(TPS65950_GPIO_BASE + 0x16)
#define TPS65950_GPIO_GPIOPUPDCTR4	(TPS65950_GPIO_BASE + 0x17)
#define TPS65950_GPIO_GPIOPUPDCTR5	(TPS65950_GPIO_BASE + 0x18)
#define TPS65950_GPIO_GPIO_ISR1A	(TPS65950_GPIO_BASE + 0x19)
#define TPS65950_GPIO_GPIO_ISR2A	(TPS65950_GPIO_BASE + 0x1a)
#define TPS65950_GPIO_GPIO_ISR3A	(TPS65950_GPIO_BASE + 0x1b)
#define TPS65950_GPIO_GPIO_IMR1A	(TPS65950_GPIO_BASE + 0x1c)
#define TPS65950_GPIO_GPIO_IMR2A	(TPS65950_GPIO_BASE + 0x1d)
#define TPS65950_GPIO_GPIO_IMR3A	(TPS65950_GPIO_BASE + 0x1e)
#define TPS65950_GPIO_GPIO_ISR1B	(TPS65950_GPIO_BASE + 0x1f)
#define TPS65950_GPIO_GPIO_ISR2B	(TPS65950_GPIO_BASE + 0x20)
#define TPS65950_GPIO_GPIO_ISR3B	(TPS65950_GPIO_BASE + 0x21)
#define TPS65950_GPIO_GPIO_IMR1B	(TPS65950_GPIO_BASE + 0x22)
#define TPS65950_GPIO_GPIO_IMR2B	(TPS65950_GPIO_BASE + 0x23)
#define TPS65950_GPIO_GPIO_IMR3B	(TPS65950_GPIO_BASE + 0x24)
#define TPS65950_GPIO_GPIO_EDR1		(TPS65950_GPIO_BASE + 0x28)
#define TPS65950_GPIO_GPIO_EDR2		(TPS65950_GPIO_BASE + 0x29)
#define TPS65950_GPIO_GPIO_EDR3		(TPS65950_GPIO_BASE + 0x2a)
#define TPS65950_GPIO_GPIO_EDR4		(TPS65950_GPIO_BASE + 0x2b)
#define TPS65950_GPIO_GPIO_EDR5		(TPS65950_GPIO_BASE + 0x2c)
#define TPS65950_GPIO_GPIO_SIH_CTRL	(TPS65950_GPIO_BASE + 0x2d)
#define TPS65950_GPIO_PMBR1		(TPS65950_GPIO_BASE + 0x2f)
#define TPS65950_GPIO_PMBR2		(TPS65950_GPIO_BASE + 0x30)

/* ID3 */
#define TPS65950_LED_BASE		0xee
#define	TPS65950_ID3_REG_LED		(TPS65950_LED_BASE + 0)
#define	TPS65950_ID3_REG_LED_LEDAON	__BIT(0)
#define	TPS65950_ID3_REG_LED_LEDBON	__BIT(1)
#define TPS65950_ID3_REG_LED_LEDAPWM	__BIT(4)
#define TPS65950_ID3_REG_LED_LEDBPWM	__BIT(5)

/* ID4 */
#define TPS65950_PM_RECEIVER_BASE	0x5b
#define TPS65950_ID4_REG_WATCHDOG_CFG	(TPS65950_PM_RECEIVER_BASE + 3)
#define TPS65950_RTC_BASE		0x1c
#define TPS65950_ID4_REG_SECONDS_REG	(TPS65950_RTC_BASE + 0)
#define TPS65950_ID4_REG_MINUTES_REG	(TPS65950_RTC_BASE + 1)
#define TPS65950_ID4_REG_HOURS_REG	(TPS65950_RTC_BASE + 2)
#define TPS65950_ID4_REG_DAYS_REG	(TPS65950_RTC_BASE + 3)
#define TPS65950_ID4_REG_MONTHS_REG	(TPS65950_RTC_BASE + 4)
#define TPS65950_ID4_REG_YEARS_REG	(TPS65950_RTC_BASE + 5)
#define TPS65950_ID4_REG_WEEKS_REG	(TPS65950_RTC_BASE + 6)
#define TPS65950_ID4_REG_RTC_CTRL_REG	(TPS65950_RTC_BASE + 13)
#define TPS65950_ID4_REG_RTC_CTRL_REG_GET_TIME __BIT(6)
#define TPS65950_ID4_REG_RTC_CTRL_REG_STOP_RTC __BIT(1)

struct tps65950_softc {
	device_t		sc_dev;
	i2c_tag_t		sc_i2c;
	i2c_addr_t		sc_addr;

	struct sysctllog	*sc_sysctllog;

	/* pih */
	void			*sc_intr;
	struct workqueue	*sc_workq;
	struct work		sc_work;
	bool			sc_queued;

#if NGPIO > 0
	/* gpio */
	struct gpio_chipset_tag	sc_gpio;
	gpio_pin_t		sc_gpio_pins[18];
	struct pic_softc	sc_gpio_pic;
#endif /* NGPIO > 0 */

#if defined(OMAP_3430)
	/* keypad */
	device_t		sc_wskbddev;
#if 0
	int			sc_spl;
#endif
	uint8_t			sc_keycodes[8];
#endif

	struct sysmon_wdog	sc_smw;
	struct todr_chip_handle	sc_todr;
};

static int	tps65950_match(device_t, cfdata_t, void *);
static void	tps65950_attach(device_t, device_t, void *);

static int	tps65950_read_1(struct tps65950_softc *, uint8_t, uint8_t *);
static int	tps65950_write_1(struct tps65950_softc *, uint8_t, uint8_t);

static void	tps65950_sysctl_attach(struct tps65950_softc *);

static int	tps65950_intr(void *);
static void	tps65950_intr_work(struct work *, void *);

static void	tps65950_pih_attach(struct tps65950_softc *, int);

static void	tps65950_bci_attach(struct tps65950_softc *);

static void	tps65950_bci_intr(struct tps65950_softc *);

#if NGPIO > 0
static void	tps65950_gpio_attach(struct tps65950_softc *, int);

static void	tps65950_gpio_intr(struct tps65950_softc *);

static int	tps65950_gpio_pin_read(void *, int);
static void	tps65950_gpio_pin_write(void *, int, int);
static void	tps65950_gpio_pin_ctl(void *, int, int);

static void	tps65950_gpio_pic_block_irqs(struct pic_softc *, size_t,
		uint32_t);
static void	tps65950_gpio_pic_unblock_irqs(struct pic_softc *, size_t,
		uint32_t);
static int	tps65950_gpio_pic_find_pending_irqs(struct pic_softc *);
static void	tps65950_gpio_pic_establish_irq(struct pic_softc *,
		struct intrsource *);

const struct pic_ops tps65950_gpio_pic_ops = {
	.pic_block_irqs = tps65950_gpio_pic_block_irqs,
	.pic_unblock_irqs = tps65950_gpio_pic_unblock_irqs,
	.pic_find_pending_irqs = tps65950_gpio_pic_find_pending_irqs,
	.pic_establish_irq = tps65950_gpio_pic_establish_irq
};
#endif /* NGPIO > 0 */

#if defined(OMAP_3430)
static void	tps65950_kbd_attach(struct tps65950_softc *);

static int	tps65950_kbd_intr(struct tps65950_softc *);

static int	tps65950_kbd_enable(void *, int);
static void	tps65950_kbd_set_leds(void *, int);
static int	tps65950_kbd_ioctl(void *, u_long, void *, int, struct lwp *);

#define KC(n)		KS_KEYCODE(n)
static const keysym_t n900_keydesc_us[] = {
	KC(0),			KS_q,
	KC(1),			KS_o,
	KC(2),			KS_p,
	KC(3),			KS_comma,
	KC(4),			KS_BackSpace,
	KC(6),			KS_a,
	KC(7),			KS_s,
	KC(8),			KS_w,
	KC(9),			KS_d,
	KC(10),			KS_f,
	KC(11),			KS_g,
	KC(12),			KS_h,
	KC(13),			KS_j,
	KC(14),			KS_k,
	KC(15),			KS_l,
	KC(16),			KS_e,
	KC(17),			KS_period,
	KC(18),			KS_Up,
	KC(19),			KS_Return,
	KC(21),			KS_z,
	KC(22),			KS_x,
	KC(23),			KS_c,
	KC(24),			KS_r,
	KC(25),			KS_v,
	KC(26),			KS_b,
	KC(27),			KS_n,
	KC(28),			KS_m,
	KC(29),			KS_space,
	KC(30),			KS_space,
	KC(31),			KS_Left,
	KC(32),			KS_t,
	KC(33),			KS_Down,
	KC(35),			KS_Right,
	KC(36),			KS_Control_L,
	KC(37),			KS_Alt_R,
	KC(38),			KS_Shift_L,
	KC(40),			KS_y,
	KC(48),			KS_u,
	KC(56),			KS_i,
	KC(57),			KS_f7,
	KC(58),			KS_f8
};

#define KBD_MAP(name, base, map) \
			{ name, base, sizeof(map)/sizeof(keysym_t), map }
const struct wscons_keydesc tps65950_kbd_keydesctab[] =
{
	KBD_MAP(KB_US,			0,	n900_keydesc_us),
	{0, 0, 0, 0}
};
#undef KBD_MAP

const struct wskbd_mapdata tps65950_kbd_keymapdata = {
	tps65950_kbd_keydesctab,
	KB_US
};

static struct wskbd_accessops tps65950_kbd_accessops = {
	tps65950_kbd_enable,
	tps65950_kbd_set_leds,
	tps65950_kbd_ioctl
};

static void	tps65950_kbd_cngetc(void *, u_int *, int *);
static void	tps65950_kbd_cnpollc(void *, int);

static const struct wskbd_consops tps65950_kbd_consops = {
	tps65950_kbd_cngetc,
	tps65950_kbd_cnpollc,
	NULL
};
#endif

static void	tps65950_rtc_attach(struct tps65950_softc *);
static int	tps65950_rtc_enable(struct tps65950_softc *, bool);
static int	tps65950_rtc_gettime(todr_chip_handle_t, struct clock_ymdhms *);
static int	tps65950_rtc_settime(todr_chip_handle_t, struct clock_ymdhms *);

static void	tps65950_wdog_attach(struct tps65950_softc *);
static int	tps65950_wdog_setmode(struct sysmon_wdog *);
static int	tps65950_wdog_tickle(struct sysmon_wdog *);

CFATTACH_DECL_NEW(tps65950pm, sizeof(struct tps65950_softc),
    tps65950_match, tps65950_attach, NULL, NULL);

static int
tps65950_match(device_t parent, cfdata_t match, void *aux)
{
	struct i2c_attach_args *ia = aux;

	switch (ia->ia_addr) {
	case TPS65950_ADDR_ID1:
	case TPS65950_ADDR_ID2:
	case TPS65950_ADDR_ID3:
	case TPS65950_ADDR_ID4:
	case TPS65950_ADDR_ID5:
		return 1;
	default:
		return 0;
	}
}

static void
tps65950_attach(device_t parent, device_t self, void *aux)
{
	struct tps65950_softc *sc = device_private(self);
	struct i2c_attach_args *ia = aux;
	uint8_t buf[4];
	uint32_t idcode;

	sc->sc_dev = self;
	sc->sc_i2c = ia->ia_tag;
	sc->sc_addr = ia->ia_addr;

	aprint_naive("\n");

	switch (sc->sc_addr) {
	case TPS65950_ADDR_ID1:
		aprint_normal("\n");
		break;
	case TPS65950_ADDR_ID2:
		memset(buf, 0, sizeof(buf));
		iic_acquire_bus(sc->sc_i2c, 0);
		tps65950_write_1(sc, TPS65950_ID2_UNLOCK_TEST_REG,
		    TPS65950_ID2_UNLOCK_TEST_REG_MAGIC);
		tps65950_read_1(sc, TPS65950_ID2_IDCODE_7_0, &buf[0]);
		tps65950_read_1(sc, TPS65950_ID2_IDCODE_15_8, &buf[1]);
		tps65950_read_1(sc, TPS65950_ID2_IDCODE_23_16, &buf[2]);
		tps65950_read_1(sc, TPS65950_ID2_IDCODE_31_24, &buf[3]);
		iic_release_bus(sc->sc_i2c, 0);
		idcode = (buf[0] << 0) | (buf[1] << 8) |
			 (buf[2] << 16) | (buf[3] << 24);
		aprint_normal(": IDCODE %08X", idcode);

		aprint_normal(", PIH");
		tps65950_pih_attach(sc, ia->ia_intr);

#if NGPIO > 0
		aprint_normal(", GPIO");
		tps65950_gpio_attach(sc, ia->ia_intrbase);
#endif /* NGPIO > 0 */

		aprint_normal("\n");
		break;
	case TPS65950_ADDR_ID3:
		aprint_normal(": LED");
		tps65950_sysctl_attach(sc);

		aprint_normal(", BCI");
		tps65950_bci_attach(sc);

#if defined(OMAP_3430)
		aprint_normal(", KEYPAD");
		tps65950_kbd_attach(sc);
#endif

		aprint_normal("\n");
		break;
	case TPS65950_ADDR_ID4:
		aprint_normal(": RTC, WATCHDOG\n");
		tps65950_rtc_attach(sc);
		tps65950_wdog_attach(sc);
		break;
	default:
		aprint_normal("\n");
		break;
	}
}

static int
tps65950_read_1(struct tps65950_softc *sc, uint8_t reg, uint8_t *val)
{
	return iic_exec(sc->sc_i2c, I2C_OP_READ_WITH_STOP, sc->sc_addr,
	    &reg, sizeof(reg), val, sizeof(*val), 0);
}

static int
tps65950_write_1(struct tps65950_softc *sc, uint8_t reg, uint8_t val)
{
	uint8_t data[2] = { reg, val };
	return iic_exec(sc->sc_i2c, I2C_OP_WRITE_WITH_STOP, sc->sc_addr,
	    NULL, 0, data, sizeof(data), 0);
}

static int
tps65950_sysctl_leda(SYSCTLFN_ARGS)
{
	struct tps65950_softc *sc;
	struct sysctlnode node;
	uint8_t val;
	u_int leda;
	int error;

	node = *rnode;
	sc = node.sysctl_data;
	iic_acquire_bus(sc->sc_i2c, 0);
	error = tps65950_read_1(sc, TPS65950_ID3_REG_LED, &val);
	iic_release_bus(sc->sc_i2c, 0);
	if (error)
		return error;
	leda = (val & TPS65950_ID3_REG_LED_LEDAON) ? 1 : 0;
	node.sysctl_data = &leda;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;

	if (leda)
		val |= (TPS65950_ID3_REG_LED_LEDAON|TPS65950_ID3_REG_LED_LEDAPWM);
	else
		val &= ~(TPS65950_ID3_REG_LED_LEDAON|TPS65950_ID3_REG_LED_LEDAPWM);

	
	iic_acquire_bus(sc->sc_i2c, 0);
	error = tps65950_write_1(sc, TPS65950_ID3_REG_LED, val);
	iic_release_bus(sc->sc_i2c, 0);

	return error;
}

static int
tps65950_sysctl_ledb(SYSCTLFN_ARGS)
{
	struct tps65950_softc *sc;
	struct sysctlnode node;
	uint8_t val;
	u_int ledb;
	int error;

	node = *rnode;
	sc = node.sysctl_data;
	iic_acquire_bus(sc->sc_i2c, 0);
	error = tps65950_read_1(sc, TPS65950_ID3_REG_LED, &val);
	iic_release_bus(sc->sc_i2c, 0);
	if (error)
		return error;
	ledb = (val & TPS65950_ID3_REG_LED_LEDBON) ? 1 : 0;
	node.sysctl_data = &ledb;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return error;

	if (ledb)
		val |= (TPS65950_ID3_REG_LED_LEDBON|TPS65950_ID3_REG_LED_LEDBPWM);
	else
		val &= ~(TPS65950_ID3_REG_LED_LEDBON|TPS65950_ID3_REG_LED_LEDBPWM);

	iic_acquire_bus(sc->sc_i2c, 0);
	error = tps65950_write_1(sc, TPS65950_ID3_REG_LED, val);
	iic_release_bus(sc->sc_i2c, 0);

	return error;
}

static void
tps65950_sysctl_attach(struct tps65950_softc *sc)
{
	struct sysctllog **log = &sc->sc_sysctllog;
	const struct sysctlnode *rnode, *cnode;
	int error;

	error = sysctl_createv(log, 0, NULL, &rnode, CTLFLAG_PERMANENT,
	    CTLTYPE_NODE, "hw", NULL, NULL, 0, NULL, 0, CTL_HW, CTL_EOL);
	if (error)
		return;

	error = sysctl_createv(log, 0, &rnode, &rnode, CTLFLAG_PERMANENT,
	    CTLTYPE_NODE, "tps65950", SYSCTL_DESCR("tps65950 control"),
	    NULL, 0, NULL, 0, CTL_CREATE, CTL_EOL);
	if (error)
		return;

	error = sysctl_createv(log, 0, &rnode, &cnode,
	    CTLFLAG_PERMANENT|CTLFLAG_READWRITE, CTLTYPE_INT, "leda",
	    SYSCTL_DESCR("LEDA enable"), tps65950_sysctl_leda, 0,
	    (void *)sc, 0, CTL_CREATE, CTL_EOL);
	if (error)
		return;

	error = sysctl_createv(log, 0, &rnode, &cnode,
	    CTLFLAG_PERMANENT|CTLFLAG_READWRITE, CTLTYPE_INT, "ledb",
	    SYSCTL_DESCR("LEDB enable"), tps65950_sysctl_ledb, 0,
	    (void *)sc, 0, CTL_CREATE, CTL_EOL);
	if (error)
		return;
}

static int
tps65950_intr(void *v)
{
	struct tps65950_softc *sc = v;

#if 1
	aprint_normal_dev(sc->sc_dev, "%s() %p %u\n", __func__, sc->sc_intr,
			sc->sc_queued);
	if (sc->sc_workq != NULL && sc->sc_queued == false) {
		workqueue_enqueue(sc->sc_workq, &sc->sc_work, NULL);
		sc->sc_queued = true;
	}
#else
	aprint_normal_dev(sc->sc_dev, "%s() %p %u\n", __func__, sc->sc_intr,
			sc->sc_queued);
	if (sc->sc_intr != NULL) {
		sc->sc_intr = intr_establish(7, IPL_VM, IST_EDGE_FALLING,
				tps65950_intr, sc);
		if (sc->sc_intr == NULL) {
			aprint_error_dev(sc->sc_dev,
					"couldn't establish interrupt\n");
		}
		sc->sc_intr = NULL;
	}
#endif

	return 1;
}

static void
tps65950_intr_work(struct work *work, void *v)
{
	struct tps65950_softc *sc = v;
	uint8_t u8;

	/* FIXME implement */
	aprint_normal_dev(sc->sc_dev, "%s()\n", __func__);

	iic_acquire_bus(sc->sc_i2c, 0);

	/* acknowledge the interrupt */
	tps65950_read_1(sc, TPS65950_PIH_REG_ISR_P1, &u8);
	aprint_normal_dev(sc->sc_dev, "%s() u8=%u\n", __func__, u8);
	tps65950_write_1(sc, TPS65950_PIH_REG_ISR_P1, u8);

	/* dispatch the interrupt */
	if (u8 & TPS65950_PIH_REG_ISR_P1_ISR0)
		tps65950_gpio_intr(sc);
	if (u8 & TPS65950_PIH_REG_ISR_P1_ISR1)
		tps65950_kbd_intr(sc);
	if (u8 & TPS65950_PIH_REG_ISR_P1_ISR2)
		tps65950_bci_intr(sc);
#if 0 /* FIXME implement */
	if (u8 & TPS65950_PIH_REG_ISR_P1_ISR3)
		tps65950_madc_intr(sc);
	if (u8 & TPS65950_PIH_REG_ISR_P1_ISR4)
		tps65950_usb_intr(sc);
	if (u8 & TPS65950_PIH_REG_ISR_P1_ISR5)
		tps65950_pm_intr(sc);
#endif

	iic_release_bus(sc->sc_i2c, 0);

	sc->sc_queued = false;
}

static void
tps65950_pih_attach(struct tps65950_softc *sc, int intr)
{
	int error;

	/* establish the interrupt handler */
	sc->sc_intr = intr_establish(intr, IPL_VM, IST_EDGE_FALLING,
			tps65950_intr, sc);
	if (sc->sc_intr == NULL) {
		aprint_error_dev(sc->sc_dev, "couldn't establish interrupt\n");
		return;
	}

	/* create the workqueue */
	error = workqueue_create(&sc->sc_workq, device_xname(sc->sc_dev),
			tps65950_intr_work, sc, 0, IPL_VM, WQ_MPSAFE);
	if (error)
		aprint_error_dev(sc->sc_dev, "couldn't create workqueue\n");
	sc->sc_queued = false;
}

static void
tps65950_bci_attach(struct tps65950_softc *sc)
{
	/* disable interrupts */
	iic_acquire_bus(sc->sc_i2c, 0);
	tps65950_write_1(sc, TPS65950_BCI_REG_BCIIMR1A, 0);
	tps65950_write_1(sc, TPS65950_BCI_REG_BCIIMR2A, 0);
	iic_release_bus(sc->sc_i2c, 0);
}

static void
tps65950_bci_intr(struct tps65950_softc *sc)
{
	uint8_t u8;

	tps65950_read_1(sc, TPS65950_BCI_REG_BCIISR1A, &u8);
	/* FIXME really implement */
	tps65950_write_1(sc, TPS65950_BCI_REG_BCIISR1A, u8);

	tps65950_read_1(sc, TPS65950_BCI_REG_BCIISR2A, &u8);
	/* FIXME really implement */
	tps65950_write_1(sc, TPS65950_BCI_REG_BCIISR2A, u8);
}

#if NGPIO > 0
static void
tps65950_gpio_attach(struct tps65950_softc *sc, int intrbase)
{
	struct gpio_chipset_tag * const gp = &sc->sc_gpio;
	struct gpiobus_attach_args gba;

	/* disable interrupts */
	iic_acquire_bus(sc->sc_i2c, 0);
	tps65950_write_1(sc, TPS65950_GPIO_GPIO_IMR1A, 0);
	tps65950_write_1(sc, TPS65950_GPIO_GPIO_IMR2A, 0);
	tps65950_write_1(sc, TPS65950_GPIO_GPIO_IMR3A, 0);
	iic_release_bus(sc->sc_i2c, 0);

	/* map interrupts */
	if (sc->sc_intr == NULL || intrbase < 0) {
		aprint_error_dev(sc->sc_dev, "couldn't map GPIO interrupts\n");
		return;
	} else {
#if 0 /* FIXME crashes */
		sc->sc_gpio_pic.pic_ops = &tps65950_gpio_pic_ops;
		strlcpy(sc->sc_gpio_pic.pic_name, device_xname(sc->sc_dev),
				sizeof(sc->sc_gpio_pic.pic_name));
		sc->sc_gpio_pic.pic_maxsources = 18;
		pic_add(&sc->sc_gpio_pic, intrbase);
		aprint_normal(": interrupts %d..%d",
				intrbase, intrbase + 17);
		/* FIXME may not be enough to map the interrupts */
#endif
	}

	gp->gp_cookie = sc;
	gp->gp_pin_read = tps65950_gpio_pin_read;
	gp->gp_pin_write = tps65950_gpio_pin_write;
	gp->gp_pin_ctl = tps65950_gpio_pin_ctl;

	gba.gba_gc = gp;
	gba.gba_pins = sc->sc_gpio_pins;
	gba.gba_npins = __arraycount(sc->sc_gpio_pins);

	/* FIXME may be missing some code here */

	config_found_ia(sc->sc_dev, "gpiobus", &gba, gpiobus_print);
}

static void
tps65950_gpio_intr(struct tps65950_softc *sc)
{
	pic_handle_intr(&sc->sc_gpio_pic);
}

static int
tps65950_gpio_pin_read(void *v, int pin)
{
	struct tps65950_softc *sc = v;
	uint8_t reg;
	uint8_t bit;
	uint8_t val;

	if (pin < 0)
		return ENODEV;
	else if (pin < 8)
	{
		reg = TPS65950_GPIO_GPIODATAIN1;
		bit = pin;
	}
	else if (pin < 16)
	{
		reg = TPS65950_GPIO_GPIODATAIN2;
		bit = pin - 8;
	}
	else if (pin < 18)
	{
		reg = TPS65950_GPIO_GPIODATAIN3;
		bit = pin - 16;
	}
	else
		return ENODEV;

	iic_acquire_bus(sc->sc_i2c, 0);
	tps65950_read_1(sc, reg, &val);
	iic_release_bus(sc->sc_i2c, 0);

	return val & (1 << bit);
}

static void
tps65950_gpio_pin_write(void *v, int pin, int value)
{
	struct tps65950_softc *sc = v;
	uint8_t reg;
	uint8_t bit;
	uint8_t val;
	uint8_t new;

	if (pin < 0)
		return;
	else if (pin < 8)
	{
		reg = TPS65950_GPIO_GPIODATAOUT1;
		bit = pin;
	}
	else if (pin < 16)
	{
		reg = TPS65950_GPIO_GPIODATAOUT2;
		bit = pin - 8;
	}
	else if (pin < 18)
	{
		reg = TPS65950_GPIO_GPIODATAOUT3;
		bit = pin - 16;
	}
	else
		return;

	iic_acquire_bus(sc->sc_i2c, 0);
	tps65950_read_1(sc, reg, &val);
	new = val;
	if (value)
		new |= (1 << bit);
	else
		new &= ~(1 << bit);
	if (new != val)
		tps65950_write_1(sc, reg, new);
	iic_release_bus(sc->sc_i2c, 0);
}

static void
tps65950_gpio_pin_ctl(void *v, int pin, int flags)
{
	struct tps65950_softc *sc = v;
	uint8_t reg;
	uint8_t bit;
	uint8_t val;
	uint8_t new;

	if (pin < 0)
		return;
	else if (pin < 8)
	{
		reg = TPS65950_GPIO_GPIODATADIR1;
		bit = pin;
	}
	else if (pin < 16)
	{
		reg = TPS65950_GPIO_GPIODATADIR2;
		bit = pin - 8;
	}
	else if (pin < 18)
	{
		reg = TPS65950_GPIO_GPIODATADIR3;
		bit = pin - 16;
	}
	else
		return;

	iic_acquire_bus(sc->sc_i2c, 0);
	tps65950_read_1(sc, reg, &val);
	new = val;
	switch (flags & (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)) {
		case GPIO_PIN_INPUT:	new &= ~(1 << bit); break;
		case GPIO_PIN_OUTPUT:	new |= (1 << bit); break;
		default:		break;
	}
	if (new != val)
		tps65950_write_1(sc, reg, new);
	iic_release_bus(sc->sc_i2c, 0);
}

static void
tps65950_gpio_pic_block_irqs(struct pic_softc *pic, size_t irq_base,
		uint32_t irq_mask)
{
	/* FIXME implement */
}

static void
tps65950_gpio_pic_unblock_irqs(struct pic_softc *pic, size_t irq_base,
		uint32_t irq_mask)
{
	/* FIXME implement */
}

static int
tps65950_gpio_pic_find_pending_irqs(struct pic_softc *pic)
{
	/* FIXME implement */
	return 0;
}

static void
tps65950_gpio_pic_establish_irq(struct pic_softc *pic, struct intrsource *is)
{
	/* FIXME implement */
}
#endif /* NGPIO > 0 */

#if defined(OMAP_3430)
static void
tps65950_kbd_attach(struct tps65950_softc *sc)
{
	uint8_t u8;
	struct wskbddev_attach_args a;

	iic_acquire_bus(sc->sc_i2c, 0);

	/* reset the keyboard */
	tps65950_write_1(sc, TPS65950_KEYPAD_REG_CTRL_REG, 0);

	/* configure the keyboard */
	u8 = TPS65950_KEYPAD_REG_CTRL_REG_KBD_ON
		| TPS65950_KEYPAD_REG_CTRL_REG_SOFTMODEN
		| TPS65950_KEYPAD_REG_CTRL_REG_SOFT_NRST;
	tps65950_write_1(sc, TPS65950_KEYPAD_REG_CTRL_REG, u8);
	u8 = 0 /* TPS65950_KEYPAD_REG_SIH_CTRL_COR */
		| TPS65950_KEYPAD_REG_SIH_CTRL_EXCLEN;
	tps65950_write_1(sc, TPS65950_KEYPAD_REG_SIH_CTRL, u8);

	iic_release_bus(sc->sc_i2c, 0);

	wskbd_cnattach(&tps65950_kbd_consops, sc, &tps65950_kbd_keymapdata);

	a.console = 1;
	a.keymap = &tps65950_kbd_keymapdata;
	a.accessops = &tps65950_kbd_accessops;
	a.accesscookie = sc;

	sc->sc_wskbddev = config_found_sm_loc(sc->sc_dev, NULL, NULL, &a,
			wskbddevprint, NULL);
}

static int
tps65950_kbd_intr(struct tps65950_softc *sc)
{
	uint8_t u8;
	uint8_t code[8];
	int i;
	int j;
	u_int type;
	int data;
	int s;

	tps65950_read_1(sc, TPS65950_KEYPAD_REG_ISR1, &u8);
	aprint_normal_dev(sc->sc_dev, "%s() %u\n", __func__, u8);

	/* check if there is anything to do */
	if (u8 == 0)
		return 0;

	/* read the keycodes pressed */
	for (i = 0; i < sizeof(code); i++) {
		/* XXX assumes registers are successive */
		tps65950_read_1(sc, TPS65950_KEYPAD_REG_FULL_CODE_7_0 + i,
				&code[i]);
	}

	/* compare with the last read */
	for (i = 0; i < sizeof(sc->sc_keycodes); i++) {
		if (sc->sc_keycodes[i] == code[i])
			continue;
		for (j = 0; j < 8; j++) {
			if ((sc->sc_keycodes[i] & (1 << j))
					== (code[i] & (1 << j)))
				continue;
			/* report the keyboard event */
			type = (code[i] & (1 << j)) ? WSCONS_EVENT_KEY_DOWN
					: WSCONS_EVENT_KEY_UP;
			data = (i * 8) + j;
			s = spltty();
			wskbd_input(sc->sc_wskbddev, type, data);
			splx(s);
		}
		sc->sc_keycodes[i] = code[i];
	}

	/* acknowledge the interrupt */
	tps65950_write_1(sc, TPS65950_KEYPAD_REG_ISR1, 0xff);

	return 1;
}

static int
tps65950_kbd_enable(void *v, int on)
{
	struct tps65950_softc *sc = v;
	uint8_t u8 = 0;

	if (sc->sc_intr == NULL)
		return ENXIO;

	iic_acquire_bus(sc->sc_i2c, 0);
	if (on)
		u8 |= TPS65950_KEYPAD_REG_IMR1_ITKPIMR1;
	else
		u8 &= ~(TPS65950_KEYPAD_REG_IMR1_ITKPIMR1);
	tps65950_write_1(sc, TPS65950_KEYPAD_REG_IMR1, u8);
	iic_release_bus(sc->sc_i2c, 0);

	return 0;
}

static void
tps65950_kbd_set_leds(void *v, int leds)
{
}

static int
tps65950_kbd_ioctl(void *v, u_long cmd, void *data, int flag, struct lwp *l)
{
	switch (cmd) {
		case WSKBDIO_GTYPE:
			*(int *)data = WSKBD_TYPE_N900;
			return 0;
		case WSKBDIO_SETLEDS:
			return 0;
		case WSKBDIO_GETLEDS:
			*(int *)data = 0;
			return 0;
	}
	return EPASSTHROUGH;
}

static void
tps65950_kbd_cngetc(void *v, u_int *type, int *data)
{
	struct tps65950_softc *sc = v;
	uint8_t u8;
	uint8_t code[8];
	int i;
	int j;

	iic_acquire_bus(sc->sc_i2c, 0);

	for (;;) {
		/* poll for keycodes */
		tps65950_read_1(sc, TPS65950_KEYPAD_REG_ISR1, &u8);
		if (u8 != 0)
			break;
		delay(100);
	}

	/* read the keycodes pressed */
	for (i = 0; i < sizeof(code); i++) {
		/* XXX assumes registers are successive */
		tps65950_read_1(sc, TPS65950_KEYPAD_REG_FULL_CODE_7_0 + i,
				&code[i]);
	}

	/* acknowledge the interrupt */
	tps65950_write_1(sc, TPS65950_KEYPAD_REG_ISR1, 0xff);

	iic_release_bus(sc->sc_i2c, 0);

	/* compare with the last read */
	for (i = 0; i < sizeof(sc->sc_keycodes); i++) {
		if (sc->sc_keycodes[i] == code[i])
			continue;
		for (j = 0; j < 8; j++) {
			if ((sc->sc_keycodes[i] & (1 << j))
					== (code[i] & (1 << j)))
				continue;
			*type = (code[i] & (1 << j)) ? WSCONS_EVENT_KEY_DOWN
					: WSCONS_EVENT_KEY_UP;
			*data = (i * 8) + j;
			sc->sc_keycodes[i] = code[i];
			break;
		}
		break;
	}
}

static void
tps65950_kbd_cnpollc(void *v, int on)
{
#if 0
	struct tps65950_softc *sc = v;

	/* FIXME tell the interrupt handler that we're polling instead?
	 * - would allow the I2C controller to keep working
	 * - would then need a way to let the workqueue get scheduled within
	 *   cngetc() */
	if (on) {
		sc->sc_spl = splbio();
	} else {
		splx(sc->sc_spl);
	}
#endif
}
#endif

static void
tps65950_rtc_attach(struct tps65950_softc *sc)
{
	sc->sc_todr.todr_gettime_ymdhms = tps65950_rtc_gettime;
	sc->sc_todr.todr_settime_ymdhms = tps65950_rtc_settime;
	sc->sc_todr.cookie = sc;
	todr_attach(&sc->sc_todr);

	iic_acquire_bus(sc->sc_i2c, 0);
	tps65950_rtc_enable(sc, true);
	iic_release_bus(sc->sc_i2c, 0);
}

static int
tps65950_rtc_enable(struct tps65950_softc *sc, bool enable)
{
	uint8_t rtc_ctrl;
	int error;

	error = tps65950_read_1(sc, TPS65950_ID4_REG_RTC_CTRL_REG, &rtc_ctrl);
	if (error)
		return error;

	if (enable) {
		rtc_ctrl |= TPS65950_ID4_REG_RTC_CTRL_REG_STOP_RTC;
	} else {
		rtc_ctrl &= ~TPS65950_ID4_REG_RTC_CTRL_REG_STOP_RTC;
	}

	return tps65950_write_1(sc, TPS65950_ID4_REG_RTC_CTRL_REG, rtc_ctrl);
}

#define RTC_READ(reg, var)						 \
	do {								 \
		tps65950_write_1(sc, TPS65950_ID4_REG_RTC_CTRL_REG,	 \
		    TPS65950_ID4_REG_RTC_CTRL_REG_GET_TIME);		 \
		if ((error = tps65950_read_1(sc, (reg), &(var))) != 0) { \
			iic_release_bus(sc->sc_i2c, 0);			 \
			return error;					 \
		}							 \
	} while (0)

#define RTC_WRITE(reg, val)						 \
	do {								 \
		if ((error = tps65950_write_1(sc, (reg), (val))) != 0) { \
			iic_release_bus(sc->sc_i2c, 0);			 \
			return error;					 \
		}							 \
	} while (0)

static int
tps65950_rtc_gettime(todr_chip_handle_t tch, struct clock_ymdhms *dt)
{
	struct tps65950_softc *sc = tch->cookie;
	uint8_t seconds_reg, minutes_reg, hours_reg,
		days_reg, months_reg, years_reg, weeks_reg;
	int error = 0;

	iic_acquire_bus(sc->sc_i2c, 0);
	RTC_READ(TPS65950_ID4_REG_SECONDS_REG, seconds_reg);
	RTC_READ(TPS65950_ID4_REG_MINUTES_REG, minutes_reg);
	RTC_READ(TPS65950_ID4_REG_HOURS_REG, hours_reg);
	RTC_READ(TPS65950_ID4_REG_DAYS_REG, days_reg);
	RTC_READ(TPS65950_ID4_REG_MONTHS_REG, months_reg);
	RTC_READ(TPS65950_ID4_REG_YEARS_REG, years_reg);
	RTC_READ(TPS65950_ID4_REG_WEEKS_REG, weeks_reg);
	iic_release_bus(sc->sc_i2c, 0);

	dt->dt_sec = FROMBCD(seconds_reg);
	dt->dt_min = FROMBCD(minutes_reg);
	dt->dt_hour = FROMBCD(hours_reg);
	dt->dt_day = FROMBCD(days_reg);
	dt->dt_mon = FROMBCD(months_reg);
	dt->dt_year = FROMBCD(years_reg) + 2000;
	dt->dt_wday = FROMBCD(weeks_reg);

	return 0;
}

static int
tps65950_rtc_settime(todr_chip_handle_t tch, struct clock_ymdhms *dt)
{
	struct tps65950_softc *sc = tch->cookie;
	int error = 0;

	iic_acquire_bus(sc->sc_i2c, 0);
	tps65950_rtc_enable(sc, false);
	RTC_WRITE(TPS65950_ID4_REG_SECONDS_REG, TOBCD(dt->dt_sec));
	RTC_WRITE(TPS65950_ID4_REG_MINUTES_REG, TOBCD(dt->dt_min));
	RTC_WRITE(TPS65950_ID4_REG_HOURS_REG, TOBCD(dt->dt_hour));
	RTC_WRITE(TPS65950_ID4_REG_DAYS_REG, TOBCD(dt->dt_day));
	RTC_WRITE(TPS65950_ID4_REG_MONTHS_REG, TOBCD(dt->dt_mon));
	RTC_WRITE(TPS65950_ID4_REG_YEARS_REG, TOBCD(dt->dt_year % 100));
	RTC_WRITE(TPS65950_ID4_REG_WEEKS_REG, TOBCD(dt->dt_wday));
	tps65950_rtc_enable(sc, true);
	iic_release_bus(sc->sc_i2c, 0);

	return error;
}

static void
tps65950_wdog_attach(struct tps65950_softc *sc)
{
	sc->sc_smw.smw_name = device_xname(sc->sc_dev);
	sc->sc_smw.smw_cookie = sc;
	sc->sc_smw.smw_setmode = tps65950_wdog_setmode;
	sc->sc_smw.smw_tickle = tps65950_wdog_tickle;
	sc->sc_smw.smw_period = TPS65950_WDOG_DEFAULT_PERIOD;

	if (sysmon_wdog_register(&sc->sc_smw) != 0)
		aprint_error_dev(sc->sc_dev, "couldn't register watchdog\n");

	iic_acquire_bus(sc->sc_i2c, 0);
	tps65950_write_1(sc, TPS65950_ID4_REG_WATCHDOG_CFG, 0);
	iic_release_bus(sc->sc_i2c, 0);
}

static int
tps65950_wdog_setmode(struct sysmon_wdog *smw)
{
	struct tps65950_softc *sc = smw->smw_cookie;
	int error;

	if ((smw->smw_mode & WDOG_MODE_MASK) == WDOG_MODE_DISARMED) {
		iic_acquire_bus(sc->sc_i2c, 0);
		error = tps65950_write_1(sc, TPS65950_ID4_REG_WATCHDOG_CFG, 0);
		iic_release_bus(sc->sc_i2c, 0);
	} else {
		if (smw->smw_period == WDOG_PERIOD_DEFAULT) {
			smw->smw_period = TPS65950_WDOG_DEFAULT_PERIOD;
		}
		if (smw->smw_period > 30) {
			error = EINVAL;
		} else {
			error = tps65950_wdog_tickle(smw);
		}
	}
	return error;
}

static int
tps65950_wdog_tickle(struct sysmon_wdog *smw)
{
	struct tps65950_softc *sc = smw->smw_cookie;
	int error;

	iic_acquire_bus(sc->sc_i2c, 0);
	tps65950_write_1(sc, TPS65950_ID4_REG_WATCHDOG_CFG, 0);
	error = tps65950_write_1(sc, TPS65950_ID4_REG_WATCHDOG_CFG,
	    smw->smw_period + 1);
	iic_release_bus(sc->sc_i2c, 0);

	return error;
}
