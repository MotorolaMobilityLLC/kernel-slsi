#include "../pmucal_common.h"
#include "../pmucal_cpu.h"
#include "../pmucal_local.h"
#include "../pmucal_rae.h"
#include "../pmucal_system.h"
#include "../pmucal_powermode.h"
#include "../pmucal_cp.h"
#include "../pmucal_gnss.h"
#include "../pmucal_shub.h"

#include "pmucal_cpu_exynos9610.h"
#include "pmucal_local_exynos9610.h"
#include "pmucal_p2vmap_exynos9610.h"
#include "pmucal_system_exynos9610.h"
#include "pmucal_cp_exynos9610.h"
#include "pmucal_gnss_exynos9610.h"
#include "pmucal_shub_exynos9610.h"

#include "cmucal-node.c"
#include "cmucal-qch.c"
#include "cmucal-sfr.c"
#include "cmucal-vclk.c"
#include "cmucal-vclklut.c"

#include "clkout_exynos9610.c"
#include "acpm_dvfs_exynos9610.h"
#include "asv_exynos9610.h"

#include "../ra.h"

void __iomem *cmu_base;
void __iomem *dll_apm_base;
void __iomem *sysreg_apm_base;
void __iomem *cmu_apm_base;

#define PLL_CON0_PLL_MMC	(0x100)
#define PLL_CON1_PLL_MMC	(0x104)
#define PLL_CON2_PLL_MMC	(0x108)
#define PLL_CON3_PLL_MMC	(0x10c)

#define PLL_ENABLE_SHIFT	(31)

#define MANUAL_MODE		(0x2)

#define MFR_MASK		(0xff)
#define MRR_MASK		(0x3f)
#define MFR_SHIFT		(16)
#define MRR_SHIFT		(24)
#define SSCG_EN			(16)

static int cmu_stable_done(void __iomem *cmu,
			unsigned char shift,
			unsigned int done,
			int usec)
{
	unsigned int result;

	do {
		result = get_bit(cmu, shift);

		if (result == done)
			return 0;
		udelay(1);
	} while (--usec > 0);

	return -EVCLKTIMEOUT;
}

int pll_mmc_enable(int enable)
{
	unsigned int reg;
	unsigned int cmu_mode;
	int ret;

	if (!cmu_base) {
		pr_err("%s: cmu_base cmuioremap failed\n", __func__);
		return -1;
	}

	cmu_mode = readl(cmu_base + PLL_CON2_PLL_MMC);
	writel(MANUAL_MODE, cmu_base + PLL_CON2_PLL_MMC);

	reg = readl(cmu_base + PLL_CON0_PLL_MMC);

	if (!enable) {
		reg &= ~(PLL_MUX_SEL);
		writel(reg, cmu_base + PLL_CON0_PLL_MMC);

		ret = cmu_stable_done(cmu_base + PLL_CON0_PLL_MMC, PLL_MUX_BUSY_SHIFT, 0, 100);

		if (ret)
			pr_err("pll mux change time out, \'PLL_MMC\'\n");
	}

	if (enable)
		reg |= 1 << PLL_ENABLE_SHIFT;
	else
		reg &= ~(1 << PLL_ENABLE_SHIFT);

	writel(reg, cmu_base + PLL_CON0_PLL_MMC);

	if (enable) {
		ret = cmu_stable_done(cmu_base + PLL_CON0_PLL_MMC, PLL_STABLE_SHIFT, 1, 100);
		if (ret)
			pr_err("pll time out, \'PLL_MMC\' %d\n", enable);

		reg |= PLL_MUX_SEL;
		writel(reg, cmu_base + PLL_CON0_PLL_MMC);

		ret = cmu_stable_done(cmu_base + PLL_CON0_PLL_MMC, PLL_MUX_BUSY_SHIFT, 0, 100);
		if (ret)
			pr_err("pll mux change time out, \'PLL_MMC\'\n");
	}

	writel(cmu_mode, cmu_base + PLL_CON2_PLL_MMC);

	return ret;
}

int cal_pll_mmc_set_ssc(unsigned int mfr, unsigned int mrr, unsigned int ssc_on)
{
	unsigned int reg;
	int ret = 0;

	ret = pll_mmc_enable(0);
	if (ret) {
		pr_err("%s: pll_mmc_disable failed\n", __func__);
		return ret;
	}

	reg = readl(cmu_base + PLL_CON3_PLL_MMC);
	reg &= ~((MFR_MASK << MFR_SHIFT) | (MRR_MASK << MRR_SHIFT));

	if (ssc_on)
		reg |= ((mfr & MFR_MASK) << MFR_SHIFT) | ((mrr & MRR_MASK) << MRR_SHIFT);

	writel(reg, cmu_base + PLL_CON3_PLL_MMC);

	reg = readl(cmu_base + PLL_CON1_PLL_MMC);

	if (ssc_on)
		reg |= 1 << SSCG_EN;
	else
		reg &= ~(1 << SSCG_EN);

	writel(reg, cmu_base + PLL_CON1_PLL_MMC);
	ret = pll_mmc_enable(1);
	if (ret)
		pr_err("%s: pll_mmc_enable failed\n", __func__);

	return ret;
}

int cal_dll_apm_enable(void)
{
	u32 timeout = 0, reg = 0;
	int ret = 0;

	if (!dll_apm_base || !sysreg_apm_base || !cmu_apm_base)
		return -EINVAL;

	/* DLL_APM_N_DCO settings */
	__raw_writel(0x1574, dll_apm_base + 0x4);

	/* DLL_APM_CTRL0 settings */
	__raw_writel(0x111, sysreg_apm_base + 0x0440);
	while (1) {
		if (__raw_readl(sysreg_apm_base + 0x0444) & 0x1)
			break;
		timeout++;
		usleep_range(10, 11);
		if (timeout > 1000) {
			pr_err("%s, timed out during dll locking\n", __func__);
			return -ETIMEDOUT;
		}
	}

	/* MUX_DLL_USER set to select CLK_DLL_DCO */
	reg = __raw_readl(cmu_apm_base + 0x0120);
	__raw_writel(reg | (0x1 << 4), cmu_apm_base + 0x0120);
	ret = cmu_stable_done(cmu_apm_base + 0x0120, PLL_MUX_BUSY_SHIFT, 0, 100);
	if (ret) {
		pr_err("MUX_DLL_USER change time out\n");
		return ret;
	}

	/* MUX_CLKCMU_SHUB_BUS set to select MUX_DLL_USER */
	reg = __raw_readl(cmu_apm_base + 0x1000);
	__raw_writel(reg | (0x1 << 0), cmu_apm_base + 0x1000);
	ret = cmu_stable_done(cmu_apm_base + 0x1000, 16, 0, 100);
	if (ret) {
		pr_err("MUX_CLKCMU_SHUB_BUS change time out\n");
		return ret;
	}

	return 0;
}

int cal_dll_set_rate(unsigned int rate)
{
	u32 timeout = 0;
	u32 n_dco = 0;

	if (!dll_apm_base || !sysreg_apm_base)
		return -EINVAL;

	n_dco = (rate / (2 * 32768)) - 1;

	/* DLL_APM_N_DCO settings */
	__raw_writel(n_dco, dll_apm_base + 0x4);

	/* Wait for DLL lock */
	while (1) {
		if (__raw_readl(sysreg_apm_base + 0x0444) & 0x1)
			break;
		timeout++;
		usleep_range(10, 11);
		if (timeout > 1000) {
			pr_err("%s, timed out during dll locking\n", __func__);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

unsigned int cal_dll_get_rate(void)
{
	u32 n_dco = 0, rate = 0;

	if (!dll_apm_base)
		return -EINVAL;

	n_dco = __raw_readl(dll_apm_base + 0x4) & 0x3fff;
	rate = (n_dco + 1) * (2 * 32768);

	return rate;
}

int cal_dll_apm_disable(void)
{
	u32 reg = 0;
	int ret = 0;

	if (!dll_apm_base || !sysreg_apm_base || !cmu_apm_base)
		return -EINVAL;

	/* MUX_CLKCMU_SHUB_BUS set to select MUX_CLKCMU_APM_BUS_USER */
	reg = __raw_readl(cmu_apm_base + 0x1000);
	__raw_writel(reg & ~(0x1 << 0), cmu_apm_base + 0x1000);
	ret = cmu_stable_done(cmu_apm_base + 0x1000, 16, 0, 100);
	if (ret) {
		pr_err("MUX_CLKCMU_SHUB_BUS change time out\n");
		return ret;
	}

	/* MUX_DLL_USER set to select OSCCLK_RCO_APM */
	reg = __raw_readl(cmu_apm_base + 0x0120);
	__raw_writel(reg & ~(0x1 << 4), cmu_apm_base + 0x0120);
	ret = cmu_stable_done(cmu_apm_base + 0x0120, PLL_MUX_BUSY_SHIFT, 0, 100);
	if (ret) {
		pr_err("MUX_DLL_USER change time out\n");
		return ret;
	}

	/* DLL_APM off */
	__raw_writel(0x0, sysreg_apm_base + 0x0440);

	return 0;
}

void exynos9610_cal_data_init(void)
{
	pr_info("%s: cal data init\n", __func__);

	cmu_base = ioremap(0x1a240000, SZ_4K);
	if (!cmu_base)
		pr_err("%s: cmu_base cmuioremap failed\n", __func__);

	dll_apm_base = ioremap(0x118b0000, SZ_4K);
	if (!dll_apm_base)
		pr_err("%s: dll_apm_base ioremap failed\n", __func__);

	sysreg_apm_base = ioremap(0x11810000, SZ_4K);
	if (!sysreg_apm_base)
		pr_err("%s: sysreg_apm_base ioremap failed\n", __func__);

	cmu_apm_base = ioremap(0x11800000, SZ_8K);
	if (!cmu_apm_base)
		pr_err("%s: cmu_apm_base ioremap failed\n", __func__);
}

void (*cal_data_init)(void) = exynos9610_cal_data_init;
