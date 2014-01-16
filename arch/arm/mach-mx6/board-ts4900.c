#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <sound/wm8962.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"

static iomux_v3_cfg_t mx6q_ts4900_pads[] = {
	/* WIFI */
	MX6Q_PAD_SD1_CMD__USDHC1_CMD,
	MX6Q_PAD_SD1_CLK__USDHC1_CLK,
	MX6Q_PAD_SD1_DAT0__USDHC1_DAT0,
	MX6Q_PAD_SD1_DAT1__USDHC1_DAT1,
	MX6Q_PAD_SD1_DAT2__USDHC1_DAT2,
	MX6Q_PAD_SD1_DAT3__USDHC1_DAT3,

	/* SD Card */
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,

	/* eMMC */
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_RST__GPIO_7_8, // EMMC_RESET#

	/* DEBUG UART */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* COM2 */
	MX6Q_PAD_GPIO_7__UART2_TXD,
	MX6Q_PAD_GPIO_8__UART2_RXD,
	MX6Q_PAD_SD4_DAT6__UART2_CTS,

	/* COM3 - NC on P1 */
	MX6Q_PAD_SD4_CMD__UART3_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,

	/* COM4 */
	MX6Q_PAD_KEY_COL0__UART4_TXD,
	MX6Q_PAD_KEY_ROW0__UART4_RXD,

	/* COM5 */
	MX6Q_PAD_KEY_COL1__UART5_TXD,
	MX6Q_PAD_KEY_ROW1__UART5_RXD,

	/* Audio */
	MX6Q_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC, // AUD_CLK
	MX6Q_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD, // AUD_TXD
	MX6Q_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS, // AUD_FRM
	MX6Q_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD, // AUD_RXD

	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */

	/* ECSPI1 */
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D19__GPIO_3_19,	/*SS1*/

	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */
	MX6Q_PAD_EIM_D23__GPIO_3_23,		/* RGMII reset */

	/* DISPLAY */
	MX6Q_PAD_SD4_DAT1__PWM3_PWMO, // Backlight
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6Q_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
};

#define TS4900_CHARGE_UOK_B	IMX_GPIO_NR(1, 27)
#define TS4900_USBH1_PWR_EN	IMX_GPIO_NR(1, 29)
#define TS4900_DISP0_PWR_EN	IMX_GPIO_NR(1, 30)

#define TS4900_SD3_CD		IMX_GPIO_NR(2, 0)
#define TS4900_SD3_WP		IMX_GPIO_NR(2, 1)
#define TS4900_SD2_CD		IMX_GPIO_NR(2, 2)
#define TS4900_SD2_WP		IMX_GPIO_NR(2, 3)
#define TS4900_CHARGE_DOK_B	IMX_GPIO_NR(2, 24)
#define TS4900_GPS_RESET	IMX_GPIO_NR(2, 28)
#define TS4900_SENSOR_EN	IMX_GPIO_NR(2, 31)

#define TS4900_GPS_EN	IMX_GPIO_NR(3, 0)
#define TS4900_DISP0_RST_B	IMX_GPIO_NR(3, 8)
#define TS4900_ALS_INT		IMX_GPIO_NR(3, 9)
#define TS4900_CHARGE_CHG_2_B	IMX_GPIO_NR(3, 13)
#define TS4900_CHARGE_FLT_2_B	IMX_GPIO_NR(3, 14)
#define TS4900_BAR0_INT	IMX_GPIO_NR(3, 15)
#define TS4900_eCOMPASS_INT	IMX_GPIO_NR(3, 16)
#define TS4900_GPS_PPS		IMX_GPIO_NR(3, 18)
#define TS4900_PCIE_PWR_EN	IMX_GPIO_NR(3, 19)
#define TS4900_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define TS4900_USB_H1_PWR	IMX_GPIO_NR(1, 29)
#define TS4900_CHARGE_CHG_1_B	IMX_GPIO_NR(3, 23)
#define TS4900_TS_INT		IMX_GPIO_NR(3, 26)
#define TS4900_DISP0_RD	IMX_GPIO_NR(3, 28)
#define TS4900_POWER_OFF	IMX_GPIO_NR(3, 29)

#define TS4900_CAN1_STBY	IMX_GPIO_NR(4, 5)
#define TS4900_ECSPI1_CS0  IMX_GPIO_NR(4, 9)
#define TS4900_CODEC_PWR_EN	IMX_GPIO_NR(4, 10)
#define TS4900_HDMI_CEC_IN	IMX_GPIO_NR(4, 11)
#define TS4900_PCIE_DIS_B	IMX_GPIO_NR(4, 14)

#define TS4900_DI0_D0_CS	IMX_GPIO_NR(5, 0)
#define TS4900_CHARGE_FLT_1_B	IMX_GPIO_NR(5, 2)
#define TS4900_PCIE_WAKE_B	IMX_GPIO_NR(5, 20)

#define TS4900_CAP_TCH_INT1	IMX_GPIO_NR(6, 7)
#define TS4900_CAP_TCH_INT0	IMX_GPIO_NR(6, 8)
#define TS4900_DISP_RST_B	IMX_GPIO_NR(6, 11)
#define TS4900_DISP_PWR_EN	IMX_GPIO_NR(6, 14)
#define TS4900_CABC_EN0	IMX_GPIO_NR(6, 15)
#define TS4900_CABC_EN1	IMX_GPIO_NR(6, 16)
#define TS4900_AUX_3V15_EN	IMX_GPIO_NR(6, 9)
#define TS4900_DISP0_WR_REVB	IMX_GPIO_NR(6, 9)
#define TS4900_AUX_5V_EN	IMX_GPIO_NR(6, 10)
#define TS4900_DI1_D0_CS	IMX_GPIO_NR(6, 31)

#define TS4900_HEADPHONE_DET	IMX_GPIO_NR(7, 8)
#define TS4900_PCIE_RST_B_REVB	IMX_GPIO_NR(7, 12)
#define TS4900_PMIC_INT_B	IMX_GPIO_NR(7, 13)
#define TS4900_PFUZE_INT	IMX_GPIO_NR(7, 13)

#define TS4900_CHARGE_NOW	IMX_GPIO_NR(1, 2)
#define TS4900_CHARGE_DONE	IMX_GPIO_NR(1, 1)
#define TS4900_ELAN_CE		IMX_GPIO_NR(2, 18)
#define TS4900_ELAN_RST	IMX_GPIO_NR(3, 8)
#define TS4900_ELAN_INT	IMX_GPIO_NR(3, 28)

#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9

static struct clk *sata_clk;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern int epdc_enabled;
extern bool enet_to_gpio_6;

static const struct esdhc_platform_data mx6q_ts4900_sd1_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct esdhc_platform_data mx6q_ts4900_sd2_data __initconst = {
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct esdhc_platform_data mx6q_ts4900_sd3_data __initconst = {
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_ts4900_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static inline void mx6q_ts4900_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
	imx6q_add_imx_uart(1, NULL);
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(3, NULL);
	imx6q_add_imx_uart(4, NULL);
}

static int mx6q_ts4900_fec_phy_init(struct phy_device *phydev)
{
	unsigned short val;

	/* Ar8031 phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);

	/* To enable AR8031 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);

	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);

	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_ts4900_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
};

static int mx6q_ts4900_spi_cs[] = {
	TS4900_ECSPI1_CS0,
};

static const struct spi_imx_master mx6q_ts4900_spi_data __initconst = {
	.chipselect     = mx6q_ts4900_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_ts4900_spi_cs),
};

#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

static struct imxi2c_platform_data mx6q_ts4900_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("wm89**", 0x1a),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
};

static void imx6q_ts4900_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(TS4900_USB_OTG_PWR, 1);
	else
		gpio_set_value(TS4900_USB_OTG_PWR, 0);
}

static void imx6q_ts4900_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(TS4900_USB_H1_PWR, 1);
	else
		gpio_set_value(TS4900_USB_H1_PWR, 0);
}

static void __init imx6q_ts4900_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(TS4900_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO TS4900_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(TS4900_USB_OTG_PWR, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(TS4900_USB_H1_PWR, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO TS4900_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(TS4900_USB_H1_PWR, 0);
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1);
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(imx6q_ts4900_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_ts4900_host1_vbus);

}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_ts4900_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_ts4900_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_ts4900_sata_data = {
	.init = mx6q_ts4900_sata_init,
	.exit = mx6q_ts4900_sata_exit,
};
#endif

static void mx6q_ts4900_flexcan0_switch(int enable)
{
	if (enable) {
		gpio_set_value(TS4900_CAN1_STBY, 1);
	} else {
		gpio_set_value(TS4900_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_ts4900_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_ts4900_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data ts4900_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 1,
	.sec_disp_id = 0,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};

static void ts4900_suspend_enter(void)
{
	/* suspend preparation */
	/* Disable AUX 5V */
	gpio_set_value(TS4900_AUX_5V_EN, 0);
}

static void ts4900_suspend_exit(void)
{
	/* resume restore */
	/* Enable AUX 5V */
	gpio_set_value(TS4900_AUX_5V_EN, 1);
}
static const struct pm_platform_data mx6q_ts4900_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = ts4900_suspend_enter,
	.suspend_exit = ts4900_suspend_exit,
};

static struct regulator_consumer_supply ts4900_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data ts4900_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(ts4900_vmmc_consumers),
	.consumer_supplies = ts4900_vmmc_consumers,
};

static struct fixed_voltage_config ts4900_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &ts4900_vmmc_init,
};

static struct platform_device ts4900_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &ts4900_vmmc_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	if (board_is_mx6_reva()) {

	} else {

	}

	return 0;
}

#ifndef CONFIG_IMX_PCIE
static void pcie_3v3_power(void)
{
	/* disable PCIE_3V3 first */
	gpio_request(TS4900_PCIE_PWR_EN, "pcie_3v3_en");
	gpio_direction_output(TS4900_PCIE_PWR_EN, 0);
	mdelay(10);
	/* enable PCIE_3V3 again */
	gpio_set_value(TS4900_PCIE_PWR_EN, 1);
	gpio_free(TS4900_PCIE_PWR_EN);
}

static void pcie_3v3_reset(void)
{
	/* reset miniPCIe */
	gpio_request(TS4900_PCIE_RST_B_REVB, "pcie_reset_rebB");
	gpio_direction_output(TS4900_PCIE_RST_B_REVB, 0);
	/* The PCI Express Mini CEM specification states that PREST# is
	deasserted minimum 1ms after 3.3vVaux has been applied and stable*/
	mdelay(1);
	gpio_set_value(TS4900_PCIE_RST_B_REVB, 1);
	gpio_free(TS4900_PCIE_RST_B_REVB);
}
#endif

static void gps_power_on(bool on)
{
	/* Enable/disable aux_3v15 */
	gpio_request(TS4900_AUX_3V15_EN, "aux_3v15_en");
	gpio_direction_output(TS4900_AUX_3V15_EN, 1);
	gpio_set_value(TS4900_AUX_3V15_EN, on);
	gpio_free(TS4900_AUX_3V15_EN);
	/*Enable/disable gps_en*/
	gpio_request(TS4900_GPS_EN, "gps_en");
	gpio_direction_output(TS4900_GPS_EN, 1);
	gpio_set_value(TS4900_GPS_EN, on);
	gpio_free(TS4900_GPS_EN);

}

static struct platform_pwm_backlight_data mx6_ts4900_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data ts4900_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

static const struct imx_pcie_platform_data mx6_ts4900_pcie_data __initconst = {
	.pcie_pwr_en	= TS4900_PCIE_PWR_EN,
	.pcie_rst	= TS4900_PCIE_RST_B_REVB,
	.pcie_wake_up	= TS4900_PCIE_WAKE_B,
	.pcie_dis	= TS4900_PCIE_DIS_B,
#ifdef CONFIG_IMX_PCIE_EP_MODE_IN_EP_RC_SYS
	.type_ep	= 1,
#else
	.type_ep	= 0,
#endif
};

/*!
 * Board specific initialization.
 */
static void __init ts4900_board_init(void)
{
	int i;
	int ret;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_ts4900_pads,
			ARRAY_SIZE(mx6q_ts4900_pads));
	} else if (cpu_is_mx6dl()) {
		// todo
	}


#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = ts4900_dvfscore_data.reg_id;
	soc_reg_id = ts4900_dvfscore_data.soc_id;
	mx6q_ts4900_init_uart();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
	}

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(ts4900_fb_data); i++)
			imx6q_add_ipuv3fb(i, &ts4900_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(ts4900_fb_data); i++)
			imx6q_add_ipuv3fb(i, &ts4900_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}

	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6q_ts4900_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_ts4900_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_ts4900_i2c_data);
	if (cpu_is_mx6dl())
		imx6q_add_imx_i2c(3, &mx6q_ts4900_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	ret = gpio_request(TS4900_PFUZE_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	} else {
		gpio_direction_input(TS4900_PFUZE_INT);
		//mx6q_ts4900_init_pfuze100(TS4900_PFUZE_INT);
	}

	imx6q_add_anatop_thermal_imx(1, &mx6q_ts4900_anatop_thermal_data);

	if (enet_to_gpio_6)
		/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
		mxc_iomux_set_specialbits_register(
			IOMUX_OBSRV_MUX1_OFFSET,
			OBSRV_MUX1_ENET_IRQ,
			OBSRV_MUX1_MASK);
	else
		fec_data.gpio_irq = -1;
	imx6_init_fec(fec_data);

	imx6q_add_pm_imx(0, &mx6q_ts4900_pm_data);

	/* Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	*/
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_ts4900_sd3_data); // emmc
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_ts4900_sd2_data); // sd
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_ts4900_sd1_data); // wifi
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_ts4900_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_ts4900_sata_data);
#else
		mx6q_ts4900_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&ts4900_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	/*
	 * Disable HannStar touch panel CABC function,
	 * this function turns the panel's backlight automatically
	 * according to the content shown on the panel which
	 * may cause annoying unstable backlight issue.
	 */
	gpio_request(TS4900_CABC_EN0, "cabc-en0");
	gpio_direction_output(TS4900_CABC_EN0, 0);
	gpio_request(TS4900_CABC_EN1, "cabc-en1");
	gpio_direction_output(TS4900_CABC_EN1, 0);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &mx6_ts4900_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&ts4900_dvfscore_data);

	/* enable sensor 3v3 and 1v8 */
	gpio_request(TS4900_SENSOR_EN, "sensor-en");
	gpio_direction_output(TS4900_SENSOR_EN, 1);

	/* enable ecompass intr */
	gpio_request(TS4900_eCOMPASS_INT, "ecompass-int");
	gpio_direction_input(TS4900_eCOMPASS_INT);
	/* enable light sensor intr */
	gpio_request(TS4900_ALS_INT, "als-int");
	gpio_direction_input(TS4900_ALS_INT);

	if (cpu_is_mx6dl()) {
		/*imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();*/
	}
	/*
	ret = gpio_request_array(mx6q_ts4900_flexcan_gpios,
			ARRAY_SIZE(mx6q_ts4900_flexcan_gpios));
	if (ret)
		pr_err("failed to request flexcan1-gpios: %d\n", ret);
	else
		imx6q_add_flexcan0(&mx6q_ts4900_flexcan0_pdata);
	*/

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	/* Enable Aux_5V */
	gpio_request(TS4900_AUX_5V_EN, "aux_5v_en");
	gpio_direction_output(TS4900_AUX_5V_EN, 1);
	gpio_set_value(TS4900_AUX_5V_EN, 1);

#ifndef CONFIG_IMX_PCIE
	/* enable pcie 3v3 power without pcie driver */
	pcie_3v3_power();
	mdelay(10);
	pcie_3v3_reset();
#endif

	gps_power_on(true);
	/* Register charger chips */
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_ts4900_pcie_data);
	if (cpu_is_mx6dl()) {
		/*mxc_iomux_v3_setup_multiple_pads(mx6dl_arm2_elan_pads,
						ARRAY_SIZE(mx6dl_arm2_elan_pads));*/
	}

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init ts4900_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer ts4900_timer = {
	.init   = ts4900_timer_init,
};

static void __init ts4900_reserve(void)
{
	phys_addr_t phys;
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)

	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
}

/*
 * initialize __mach_desc_MX6Q_TS4900 data structure.
 */
MACHINE_START(TS4900, "Freescale i.MX 6Quad TS-4900 Board")
	/* Maintainer: Technologic Systems */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = ts4900_board_init,
	.timer = &ts4900_timer,
	.reserve = ts4900_reserve,
MACHINE_END
