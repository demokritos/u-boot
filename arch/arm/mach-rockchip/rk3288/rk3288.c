// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016 Rockchip Electronics Co., Ltd
 */
#include <common.h>
#include <dm.h>
#include <env.h>
#include <clk.h>
#include <init.h>
#include <asm/armv7.h>
#include <asm/io.h>
#include <asm/arch-rockchip/bootrom.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/cru_rk3288.h>
#include <asm/arch-rockchip/hardware.h>
#include <asm/arch-rockchip/grf_rk3288.h>
#include <asm/arch-rockchip/pmu_rk3288.h>
#include <asm/arch-rockchip/qos_rk3288.h>
#include <asm/arch-rockchip/sdram.h>
#include <asm/arch-rockchip/gpio.h>

DECLARE_GLOBAL_DATA_PTR;

#define GRF_BASE	0xff770000

const char * const boot_devices[BROM_LAST_BOOTSOURCE + 1] = {
	[BROM_BOOTSOURCE_EMMC] = "/dwmmc@ff0f0000",
	[BROM_BOOTSOURCE_SD] = "/dwmmc@ff0c0000",
};

enum project_id {
	TinkerBoardS = 0,
	TinkerBoard  = 7,
};

enum pcb_id {
	SR,
	ER,
	PR,
};

extern bool force_ums;

#ifdef CONFIG_SPL_BUILD
static void configure_l2ctlr(void)
{
	u32 l2ctlr;

	l2ctlr = read_l2ctlr();
	l2ctlr &= 0xfffc0000; /* clear bit0~bit17 */

	/*
	 * Data RAM write latency: 2 cycles
	 * Data RAM read latency: 2 cycles
	 * Data RAM setup latency: 1 cycle
	 * Tag RAM write latency: 1 cycle
	 * Tag RAM read latency: 1 cycle
	 * Tag RAM setup latency: 1 cycle
	 */
	l2ctlr |= (1 << 3 | 1 << 0);
	write_l2ctlr(l2ctlr);
}
#endif

int rk3288_qos_init(void)
{
	int val = 2 << PRIORITY_HIGH_SHIFT | 2 << PRIORITY_LOW_SHIFT;
	/* set vop qos to higher priority */
	writel(val, CPU_AXI_QOS_PRIORITY + VIO0_VOP_QOS);
	writel(val, CPU_AXI_QOS_PRIORITY + VIO1_VOP_QOS);

	if (!fdt_node_check_compatible(gd->fdt_blob, 0,
				       "rockchip,rk3288-tinker")) {
		/* set isp qos to higher priority */
		writel(val, CPU_AXI_QOS_PRIORITY + VIO1_ISP_R_QOS);
		writel(val, CPU_AXI_QOS_PRIORITY + VIO1_ISP_W0_QOS);
		writel(val, CPU_AXI_QOS_PRIORITY + VIO1_ISP_W1_QOS);
	}

	return 0;
}

int arch_cpu_init(void)
{
#ifdef CONFIG_SPL_BUILD
	configure_l2ctlr();
#else
	/* We do some SoC one time setting here. */
	struct rk3288_grf * const grf = (void *)GRF_BASE;

	/* Use rkpwm by default */
	rk_setreg(&grf->soc_con2, 1 << 0);

	/*
	 * Disable JTAG on sdmmc0 IO. The SDMMC won't work until this bit is
	 * cleared
	 */
	rk_clrreg(&grf->soc_con0, 1 << 12);

	rk3288_qos_init();
#endif

	return 0;
}

#ifdef CONFIG_DEBUG_UART_BOARD_INIT
void board_debug_uart_init(void)
{
	/* Enable early UART on the RK3288 */
	struct rk3288_grf * const grf = (void *)GRF_BASE;

	rk_clrsetreg(&grf->gpio7ch_iomux, GPIO7C7_MASK << GPIO7C7_SHIFT |
		     GPIO7C6_MASK << GPIO7C6_SHIFT,
		     GPIO7C7_UART2DBG_SOUT << GPIO7C7_SHIFT |
		     GPIO7C6_UART2DBG_SIN << GPIO7C6_SHIFT);
}
#endif

static void rk3288_detect_reset_reason(void)
{
	struct rk3288_cru *cru = rockchip_get_cru();
	const char *reason;

	if (IS_ERR(cru))
		return;

	switch (cru->cru_glb_rst_st) {
	case GLB_POR_RST:
		reason = "POR";
		break;
	case FST_GLB_RST_ST:
	case SND_GLB_RST_ST:
		reason = "RST";
		break;
	case FST_GLB_TSADC_RST_ST:
	case SND_GLB_TSADC_RST_ST:
		reason = "THERMAL";
		break;
	case FST_GLB_WDT_RST_ST:
	case SND_GLB_WDT_RST_ST:
		reason = "WDOG";
		break;
	default:
		reason = "unknown reset";
	}

	env_set("reset_reason", reason);

	/*
	 * Clear cru_glb_rst_st, so we can determine the last reset cause
	 * for following resets.
	 */
	rk_clrreg(&cru->cru_glb_rst_st, GLB_RST_ST_MASK);
}

__weak int rk3288_board_late_init(void)
{
	return 0;
}

int rk_board_late_init(void)
{
	rk3288_detect_reset_reason();

	return rk3288_board_late_init();
}

static int do_clock(cmd_tbl_t *cmdtp, int flag, int argc,
		       char * const argv[])
{
	static const struct {
		char *name;
		int id;
	} clks[] = {
		{ "osc", CLK_OSC },
		{ "apll", CLK_ARM },
		{ "dpll", CLK_DDR },
		{ "cpll", CLK_CODEC },
		{ "gpll", CLK_GENERAL },
#ifdef CONFIG_ROCKCHIP_RK3036
		{ "mpll", CLK_NEW },
#else
		{ "npll", CLK_NEW },
#endif
	};
	int ret, i;
	struct udevice *dev;

	ret = rockchip_get_clk(&dev);
	if (ret) {
		printf("clk-uclass not found\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(clks); i++) {
		struct clk clk;
		ulong rate;

		clk.id = clks[i].id;
		ret = clk_request(dev, &clk);
		if (ret < 0)
			continue;

		rate = clk_get_rate(&clk);
		printf("%s: %lu\n", clks[i].name, rate);

		clk_free(&clk);
	}

	return 0;
}

/*
*
* usb current limit : GPIO6_A6 (H:unlock, L:lock)
*
*/
void usb_current_limit_ctrl(bool unlock_current)
{
	int tmp;

	printf("%s: unlock_current = %d\n", __func__, unlock_current);
	tmp = readl(RKIO_GPIO6_PHYS + GPIO_SWPORT_DR);
	if(unlock_current == true)
		writel(tmp | 0x40, RKIO_GPIO6_PHYS + GPIO_SWPORT_DR);
	else
		writel(tmp & ~0x40, RKIO_GPIO6_PHYS + GPIO_SWPORT_DR);

	tmp = readl(RKIO_GPIO6_PHYS + GPIO_SWPORT_DDR);
	writel(tmp | 0x40, RKIO_GPIO6_PHYS + GPIO_SWPORT_DDR);
}

/*
*
* eMMC maskrom mode : GPIO6_A7 (H:disable maskrom, L:enable maskrom)
*
*/
void rk3288_maskrom_ctrl(bool enable_emmc)
{
	int tmp;

	printf("%s: enable_emmc = %d\n", __func__, enable_emmc);
	tmp = readl(RKIO_GPIO6_PHYS + GPIO_SWPORT_DR);
	if(enable_emmc == true)
		writel(tmp | 0x80, RKIO_GPIO6_PHYS + GPIO_SWPORT_DR);
	else
		writel(tmp & ~0x80, RKIO_GPIO6_PHYS + GPIO_SWPORT_DR);

	tmp = readl(RKIO_GPIO6_PHYS + GPIO_SWPORT_DDR);
	writel(tmp | 0x80, RKIO_GPIO6_PHYS + GPIO_SWPORT_DDR);
	mdelay(10);
}

/*
*
* project id        : GPIO2_A3 GPIO2_A2 GPIO2_A1
* pcb id            : GPIO2_B2 GPIO2_B1 GPIO2_B0
* SDP/CDP           : GPIO6_A5 (H:SDP, L:CDP)
* usb current limit : GPIO6_A6 (H:unlock, L:lock)
* eMMC maskrom mode : GPIO6_A7 (H:disable maskrom, L:enable maskrom)
*
* Please check TRM V1.2 part1 page 152 for the following register settings
*
*/
int check_force_enter_ums_mode(void)
{
	int tmp;
	enum pcb_id pcbid;
	enum project_id projectid;

	// GPIO2_A3/GPIO2_A2/GPIO2_A1 pull up enable
	tmp = readl(RKIO_GRF_PHYS + GRF_GPIO2A_P);
	writel((tmp&~(0x03F<<2)) | 0x3F<<(16 + 2) | 0x15<<2, RKIO_GRF_PHYS + GRF_GPIO2A_P);

	// GPIO2_A3/GPIO2_A2/GPIO2_A1/GPIO2_B2/GPIO2_B1/GPIO2_B0 set to input
	tmp = readl(RKIO_GPIO2_PHYS + GPIO_SWPORT_DDR);
	writel(tmp & ~(0x70E), RKIO_GPIO2_PHYS + GPIO_SWPORT_DDR);

	// GPIO6_A5 pull up/down disable
	tmp = readl(RKIO_GRF_PHYS + GRF_GPIO6A_P);
	writel((tmp&~(0x03<<10)) | 0x03<<(16 + 10), RKIO_GRF_PHYS + GRF_GPIO6A_P);

	// GPIO6_A5 set to input
	tmp = readl(RKIO_GPIO6_PHYS + GPIO_SWPORT_DDR);
	writel(tmp & ~(0x20), RKIO_GPIO6_PHYS + GPIO_SWPORT_DDR);

	mdelay(10);

	// read GPIO2_A3/GPIO2_A2/GPIO2_A1 value
	projectid = (readl(RKIO_GPIO2_PHYS + GPIO_EXT_PORT) & 0x0E) >>1;

	// read GPIO2_B2/GPIO2_B1/GPIO2_B0 value
	pcbid = (readl(RKIO_GPIO2_PHYS + GPIO_EXT_PORT) & 0x700) >> 8;

	// only Tinker Board S and the PR stage PCB has this function
	if(projectid!=TinkerBoard && pcbid >= ER){
		printf("PC event = 0x%x\n", readl(RKIO_GPIO6_PHYS + GPIO_EXT_PORT)&0x20);
		if((readl(RKIO_GPIO6_PHYS + GPIO_EXT_PORT)&0x20)==0x20) {
			// SDP detected, enable EMMC and unlock usb current limit
			printf("usb connected to SDP, force enter ums mode\n");
			force_ums = true;
			rk3288_maskrom_ctrl(true);
			usb_current_limit_ctrl(true);
		} else {
			usb_current_limit_ctrl(false);
		}
	}
	return 0;
}

U_BOOT_CMD(
	clock, 2, 1, do_clock,
	"display information about clocks",
	""
);
