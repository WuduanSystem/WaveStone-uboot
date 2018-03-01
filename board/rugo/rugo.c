/*
 * Copyright (C) 2017 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx7-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
//#include <miiphy.h>
//#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "../freescale/common/pfuze.h"
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/arch/crm_regs.h>
#include <usb.h>
#include <malloc.h>


DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_DSE_3P3V_49OHM | PAD_CTL_PUS_PU100KOHM | \
			PAD_CTL_HYS)
#define USDHC_PAD_CTRL (PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW |	\
			PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU47KOHM)

#define I2C_PAD_CTRL	(PAD_CTL_DSE_3P3V_32OHM | PAD_CTL_SRE_SLOW | \
			PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_PU100KOHM)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_PU100KOHM | \
			PAD_CTL_DSE_3P3V_49OHM)
			


#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX7D_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = MX7D_PAD_I2C1_SCL__GPIO4_IO8 | PC,
		.gp = IMX_GPIO_NR(4, 8),
	},
	.sda = {
		.i2c_mode = MX7D_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = MX7D_PAD_I2C1_SDA__GPIO4_IO9 | PC,
		.gp = IMX_GPIO_NR(4, 9),
	},
};
#endif

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

static iomux_v3_cfg_t const wdog_pads[] = {
	MX7D_PAD_GPIO1_IO00__WDOG1_WDOG_B | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const uart1_pads[] = {
	MX7D_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX7D_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc3_emmc_pads[] = {
	MX7D_PAD_SD3_CLK__SD3_CLK     | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_CMD__SD3_CMD     | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_DATA7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD3_RESET_B__SD3_RESET_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX7D_PAD_SD2_CLK__SD2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD2_CMD__SD2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD2_DATA0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD2_DATA1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD2_DATA2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX7D_PAD_SD2_DATA3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/*CD pin, to do need to find a correct pin*/
	//MX7D_PAD_SD1_CD_B__GPIO5_IO0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/*RESET*/
	MX7D_PAD_SD2_RESET_B__GPIO5_IO11 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

//#define USDHC2_PWR_GPIO IMX_GPIO_NR(5, 11)

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
};

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC3_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	// case USDHC2_BASE_ADDR:
	// 	ret = 1; 	no cd pin, assume uSDHC2 sd is always present
	// 	//ret = !gpio_get_value(USDHC2_CD_GPIO);
	// 	break;
	case USDHC3_BASE_ADDR:
		ret = 1; /* Assume uSDHC3 emmc is always present */
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	int i,ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc1                    USDHC2
	 * mmc2                    USDHC3 (eMMC)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc3_emmc_pads, ARRAY_SIZE(usdhc3_emmc_pads));
			
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		// case 1:
		// 	imx_iomux_v3_setup_multiple_pads(
		// 		usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
		// 	//gpio_request(USDHC2_CD_GPIO, "usdhc2_cd");
		// 	//gpio_direction_input(USDHC2_CD_GPIO);
		// 	gpio_request(USDHC2_PWR_GPIO, "usdhc2_pwr");
		// 	gpio_direction_output(USDHC2_PWR_GPIO, 0);
		// 	udelay(500);
		// 	gpio_direction_output(USDHC2_PWR_GPIO, 1);
		// 	usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		// 	break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
			}

			ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
			if (ret)
				return ret;
		}
	return 0;
}

/*Init 74LV595*/

#define IOX_SDI IMX_GPIO_NR(1, 9)
#define IOX_STCP IMX_GPIO_NR(1, 12)
#define IOX_SHCP IMX_GPIO_NR(1, 13)

static iomux_v3_cfg_t const iox_pads[] = {
	/* IOX_SDI */
	MX7D_PAD_GPIO1_IO09__GPIO1_IO9	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* IOX_STCP */
	MX7D_PAD_GPIO1_IO12__GPIO1_IO12	| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* IOX_SHCP */
	MX7D_PAD_GPIO1_IO13__GPIO1_IO13	| MUX_PAD_CTRL(NO_PAD_CTRL),
};

/*
 * PCIE_DIS_B --> Q0
 * PCIE_RST_B --> Q1
 * HDMI_RST_B --> Q2
 * PERI_RST_B --> Q3
 * SENSOR_RST_B --> Q4
 * ENET_RST_B --> Q5
 * PERI_3V3_EN --> Q6
 * LCD_PWR_EN --> Q7
 */
enum qn {
	PCIE_DIS_B,
	PCIE_RST_B,
	HDMI_RST_B,
	PERI_RST_B,
	SENSOR_RST_B,
	ENET_RST_B,
	PERI_3V3_EN,
	LCD_PWR_EN,
};

enum qn_func {
	qn_reset,
	qn_enable,
	qn_disable,
};

enum qn_level {
	qn_low = 0,
	qn_high = 1,
};

static enum qn_level seq[3][2] = {
	{0, 1}, {1, 1}, {0, 0}
};

static enum qn_func qn_output[8] = {
	qn_disable, qn_reset, qn_reset, qn_reset, qn_reset, qn_reset, qn_enable,
	qn_disable
};

static void iox74lv_init(void)
{
	int i;

	for (i = 7; i >= 0; i--) {
		gpio_direction_output(IOX_SHCP, 0);
		gpio_direction_output(IOX_SDI, seq[qn_output[i]][0]);
		udelay(500);
		gpio_direction_output(IOX_SHCP, 1);
		udelay(500);
	}

	gpio_direction_output(IOX_STCP, 0);
	udelay(500);
	/*
	  * shift register will be output to pins
	  */
	gpio_direction_output(IOX_STCP, 1);

	for (i = 7; i >= 0; i--) {
		gpio_direction_output(IOX_SHCP, 0);
		gpio_direction_output(IOX_SDI, seq[qn_output[i]][1]);
		udelay(500);
		gpio_direction_output(IOX_SHCP, 1);
		udelay(500);
	}
	gpio_direction_output(IOX_STCP, 0);
	udelay(500);
	/*
	  * shift register will be output to pins
	  */
	gpio_direction_output(IOX_STCP, 1);
};

#ifdef CONFIG_VIDEO_MXS
static iomux_v3_cfg_t const lcd_pads[] = {
	MX7D_PAD_LCD_CLK__LCD_CLK | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_ENABLE__LCD_ENABLE | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_HSYNC__LCD_HSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_VSYNC__LCD_VSYNC | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA00__LCD_DATA0 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA01__LCD_DATA1 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA02__LCD_DATA2 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA03__LCD_DATA3 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA04__LCD_DATA4 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA05__LCD_DATA5 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA06__LCD_DATA6 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA07__LCD_DATA7 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA08__LCD_DATA8 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA09__LCD_DATA9 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA10__LCD_DATA10 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA11__LCD_DATA11 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA12__LCD_DATA12 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA13__LCD_DATA13 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA14__LCD_DATA14 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA15__LCD_DATA15 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA16__LCD_DATA16 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA17__LCD_DATA17 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA18__LCD_DATA18 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA19__LCD_DATA19 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA20__LCD_DATA20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA21__LCD_DATA21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA22__LCD_DATA22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX7D_PAD_LCD_DATA23__LCD_DATA23 | MUX_PAD_CTRL(LCD_PAD_CTRL),

	MX7D_PAD_LCD_RESET__GPIO3_IO4	| MUX_PAD_CTRL(LCD_PAD_CTRL),
};

static iomux_v3_cfg_t const pwm_pads[] = {
	/* Use GPIO for Brightness adjustment, duty cycle = period */
	MX7D_PAD_GPIO1_IO01__GPIO1_IO1 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static int setup_lcd(void)
{
	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));

	imx_iomux_v3_setup_multiple_pads(pwm_pads, ARRAY_SIZE(pwm_pads));

	/* Reset LCD */
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 0);//gpio_direction_output(64+4,0)
	udelay(500);
	gpio_direction_output(IMX_GPIO_NR(3, 4) , 1);

	/* Set Brightness to high */
	gpio_direction_output(IMX_GPIO_NR(1, 1) , 1);

	return 0;
}
#endif



int board_early_init_f(void)
{
	setup_iomux_uart();
	
#ifdef CONFIG_SYS_I2C_MXC
		setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
#endif

	return 0;
}

#ifdef CONFIG_POWER
#define I2C_PMIC       0
static struct pmic *pfuze;
int power_init_board(void)
{
	int ret;
	unsigned int reg, rev_id;

	ret = power_pfuze3000_init(I2C_PMIC);
	if (ret)
		return ret;

	pfuze = pmic_get("PFUZE3000");
	ret = pmic_probe(pfuze);
	if (ret)
		return ret;

	pmic_reg_read(pfuze, PFUZE3000_DEVICEID, &reg);
	pmic_reg_read(pfuze, PFUZE3000_REVID, &rev_id);
	printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", reg, rev_id);

	/* disable Low Power Mode during standby mode */
	pmic_reg_write(pfuze, PFUZE3000_LDOGCTL, 0x1);
	return 0;
}
#endif

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	imx_iomux_v3_setup_multiple_pads(iox_pads, ARRAY_SIZE(iox_pads));

	iox74lv_init();

#ifdef CONFIG_VIDEO_MXS
	setup_lcd();
#endif

	return 0;
}

int checkboard(void)
{
	char *mode;
	if(IS_ENABLED(CONFIG_ARMV7_BOOT_SEC_DEFAULT))
		mode = "secure";
	else
		mode = "non-secure";
	printf("Board: Rugo in %s\n",mode);

	return 0;
}

int board_usb_phy_mode(int port)
{
	return USB_INIT_DEVICE;
}

int board_late_init(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(wdog_pads, ARRAY_SIZE(wdog_pads));

	set_wdog_reset(wdog);

	/*
	 * Do not assert internal WDOG_RESET_B_DEB(controlled by bit 4),
	 * since we use PMIC_PWRON to reset the board.
	 */
	clrsetbits_le16(&wdog->wcr, 0, 0x10);

	return 0;
}
