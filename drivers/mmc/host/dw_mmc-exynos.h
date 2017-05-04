/*
 * Exynos Specific Extensions for Synopsys DW Multimedia Card Interface driver
 *
 * Copyright (C) 2012-2014 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _DW_MMC_EXYNOS_H_
#define _DW_MMC_EXYNOS_H_

/* Exynos implementation specific driver private data */
struct dw_mci_exynos_priv_data {
	u8			ctrl_type;
	u8			ciu_div;
	u32			sdr_timing;
	u32			ddr_timing;
	u32			hs400_timing;
	u32			tuned_sample;
	u32			cur_speed;
	u32			dqs_delay;
	u32			saved_dqs_en;
	u32			saved_strobe_ctrl;
	u32			hs200_timing;
	u32			ddr200_timing;
	u32			ddr200_ulp_timing;
	u32			ddr200_tx_t_fastlimit;
	u32			ddr200_tx_t_initval;
	u32			sdr104_timing;
	u32			sdr50_timing;
	u32			*ref_clk;
	u32			delay_line;
	u32			tx_delay_line;
	struct pinctrl		*pinctrl;
	u32			clk_drive_number;
	u32			clk_drive_tuning;
	struct pinctrl_state	*clk_drive_base;
	struct pinctrl_state	*clk_drive_str[6];
	int			cd_gpio;
	u32			caps;
	u32			ctrl_flag;
	u32			ctrl_windows;
	u32			ignore_phase;
	u32			selclk_drv;
	u32			voltage_int_extra;

#define DW_MMC_EXYNOS_USE_FINE_TUNING		BIT(0)
#define DW_MMC_EXYNOS_BYPASS_FOR_ALL_PASS	BIT(1)
#define DW_MMC_EXYNOS_ENABLE_SHIFT		BIT(2)
};

#define SDMMC_CLKSEL			0x09C
#define SDMMC_CLKSEL64			0x0A8

/* Extended Register's Offset */
#define SDMMC_HS400_DQS_EN		0x180
#define SDMMC_HS400_ASYNC_FIFO_CTRL	0x184
#define SDMMC_HS400_DLINE_CTRL		0x188

/* CLKSEL register defines */
#define SDMMC_CLKSEL_CCLK_SAMPLE(x)	(((x) & 7) << 0)
#define SDMMC_CLKSEL_CCLK_FINE_SAMPLE(x)	(((x) & 0xF) << 0)
#define SDMMC_CLKSEL_CCLK_DRIVE(x)	(((x) & 7) << 16)
#define SDMMC_CLKSEL_CCLK_FINE_DRIVE(x)	(((x) & 3) << 22)
#define SDMMC_CLKSEL_CCLK_DIVIDER(x)	(((x) & 7) << 24)
#define SDMMC_CLKSEL_GET_DRV_WD3(x)	(((x) >> 16) & 0x7)
#define SDMMC_CLKSEL_GET_DIV(x)		(((x) >> 24) & 0x7)
#define SDMMC_CLKSEL_GET_DIVRATIO(x)	((((x) >> 24) & 0x7) + 1)
#define SDMMC_CLKSEL_UP_SAMPLE(x, y)	(((x) & ~SDMMC_CLKSEL_CCLK_SAMPLE(7)) |\
					 SDMMC_CLKSEL_CCLK_SAMPLE(y))
#define SDMMC_CLKSEL_TIMING(div, f_drv, drv, sample) \
	(SDMMC_CLKSEL_CCLK_DIVIDER(div) |	\
	 SDMMC_CLKSEL_CCLK_FINE_DRIVE(f_drv) |	\
	 SDMMC_CLKSEL_CCLK_DRIVE(drv) |		\
	 SDMMC_CLKSEL_CCLK_SAMPLE(sample))
#define SDMMC_CLKSEL_TIMING_MASK	SDMMC_CLKSEL_TIMING(0x7, 0x7, 0x7, 0x7)
#define SDMMC_CLKSEL_WAKEUP_INT		BIT(11)

/* RCLK_EN register defines */
#define DATA_STROBE_EN			BIT(0)
#define AXI_NON_BLOCKING_WR	BIT(7)

/* SDMMC_DDR200_RDDQS_EN */
#define DWMCI_TXDT_CRC_TIMER_FASTLIMIT(x)	(((x) & 0xFF) << 16)
#define DWMCI_TXDT_CRC_TIMER_INITVAL(x)		(((x) & 0xFF) << 8)
#define DWMCI_TXDT_CRC_TIMER_SET(x, y)	(DWMCI_TXDT_CRC_TIMER_FASTLIMIT(x) | \
					DWMCI_TXDT_CRC_TIMER_INITVAL(y))
#define DWMCI_AXI_NON_BLOCKING_WRITE		BIT(7)
#define DWMCI_RESP_RCLK_MODE			BIT(5)
#define DWMCI_BUSY_CHK_CLK_STOP_EN		BIT(2)
#define DWMCI_RXDATA_START_BIT_SEL		BIT(1)
#define DWMCI_RDDQS_EN				BIT(0)
#define DWMCI_DDR200_RDDQS_EN_DEF	(DWMCI_TXDT_CRC_TIMER_FASTLIMIT(0x13) | \
					DWMCI_TXDT_CRC_TIMER_INITVAL(0x15))

/* SDMMC_DDR200_ASYNC_FIFO_CTRL */
#define DWMCI_ASYNC_FIFO_RESET		BIT(0)

/* SDMMC_DDR200_DLINE_CTRL */
#define DWMCI_WD_DQS_DELAY_CTRL(x)		(((x) & 0x3FF) << 20)
#define DWMCI_FIFO_CLK_DELAY_CTRL(x)		(((x) & 0x3) << 16)
#define DWMCI_RD_DQS_DELAY_CTRL(x)		((x) & 0x3FF)
#define DWMCI_DDR200_DLINE_CTRL_SET(x, y, z)	(DWMCI_WD_DQS_DELAY_CTRL(x) | \
						DWMCI_FIFO_CLK_DELAY_CTRL(y) | \
						DWMCI_RD_DQS_DELAY_CTRL(z))
#define DWMCI_DDR200_DLINE_CTRL_DEF	(DWMCI_FIFO_CLK_DELAY_CTRL(0x2) | \
					DWMCI_RD_DQS_DELAY_CTRL(0x40))

/* DLINE_CTRL register defines */
#define DQS_CTRL_RD_DELAY(x, y)		(((x) & ~0x3FF) | ((y) & 0x3FF))
#define DQS_CTRL_GET_RD_DELAY(x)	((x) & 0x3FF)

/* Protector Register */
#define SDMMC_EMMCP_BASE	0x1000
#define SDMMC_MPSECURITY	(SDMMC_EMMCP_BASE + 0x0010)
#define SDMMC_MPSBEGIN0		(SDMMC_EMMCP_BASE + 0x0200)
#define SDMMC_MPSEND0		(SDMMC_EMMCP_BASE + 0x0204)
#define SDMMC_MPSCTRL0		(SDMMC_EMMCP_BASE + 0x020C)

/* SMU control defines */
#define SDMMC_MPSCTRL_SECURE_READ_BIT		BIT(7)
#define SDMMC_MPSCTRL_SECURE_WRITE_BIT		BIT(6)
#define SDMMC_MPSCTRL_NON_SECURE_READ_BIT	BIT(5)
#define SDMMC_MPSCTRL_NON_SECURE_WRITE_BIT	BIT(4)
#define SDMMC_MPSCTRL_USE_FUSE_KEY		BIT(3)
#define SDMMC_MPSCTRL_ECB_MODE			BIT(2)
#define SDMMC_MPSCTRL_ENCRYPTION		BIT(1)
#define SDMMC_MPSCTRL_VALID			BIT(0)

/* Maximum number of Ending sector */
#define SDMMC_ENDING_SEC_NR_MAX	0xFFFFFFFF

/* Fixed clock divider */
#define EXYNOS4210_FIXED_CIU_CLK_DIV	2
#define EXYNOS4412_FIXED_CIU_CLK_DIV	4
#define HS400_FIXED_CIU_CLK_DIV		1

/* Minimal required clock frequency for cclkin, unit: HZ */
#define EXYNOS_CCLKIN_MIN	50000000

#endif /* _DW_MMC_EXYNOS_H_ */
