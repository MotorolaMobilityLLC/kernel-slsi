#include <linux/io.h>
#include <linux/pinctrl/consumer.h>

#include "vipx-config.h"
#include "vipx-exynos.h"

extern struct vipx_clk vipx_clk_array[];
extern const u32 vipx_clk_array_size;
extern const struct vipx_clk_ops vipx_clk_ops;
extern const struct vipx_ctl_ops vipx_ctl_ops;

int vipx_clk_set_rate(struct device *dev, u32 index, ulong frequency)
{
	int ret = 0;
	struct clk *clk;

	if (index >= vipx_clk_array_size) {
		vipx_err("index is invalid(%d >= %d)\n", index, vipx_clk_array_size);
		ret = -EINVAL;
		goto p_err;
	}

	clk= vipx_clk_array[index].clk;
	if (IS_ERR_OR_NULL(clk)) {
		vipx_err("clk is NULL(%d)\n", index);
		ret = -EINVAL;
		goto p_err;
	}

	ret = clk_set_rate(clk, frequency);
	if (ret) {
		vipx_err("clk_set_rate is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

ulong vipx_clk_get_rate(struct device *dev, u32 index)
{
	ulong frequency;
	struct clk *clk;

	if (index >= vipx_clk_array_size) {
		vipx_err("index is invalid(%d >= %d)\n", index, vipx_clk_array_size);
		frequency = -EINVAL;
		goto p_err;
	}

	clk = vipx_clk_array[index].clk;
	if (IS_ERR_OR_NULL(clk)) {
		vipx_err("clk is NULL(%d)\n", index);
		frequency = -EINVAL;
		goto p_err;
	}

	frequency = clk_get_rate(clk);

p_err:
	vipx_info("%s : %ldMhz\n", vipx_clk_array[index].name, frequency/1000000);
	return frequency;
}

int  vipx_clk_enable(struct device *dev, u32 index)
{
	int ret = 0;
	struct clk *clk;

	if (index >= vipx_clk_array_size) {
		vipx_err("index is invalid(%d >= %d)\n", index, vipx_clk_array_size);
		ret = -EINVAL;
		goto p_err;
	}

	clk = vipx_clk_array[index].clk;
	if (IS_ERR_OR_NULL(clk)) {
		vipx_err("clk is NULL(%d)\n", index);
		ret = -EINVAL;
		goto p_err;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		vipx_err("clk_prepare_enable is fail(%s)\n", vipx_clk_array[index].name);
		goto p_err;
	}

p_err:
	return ret;
}

int vipx_clk_disable(struct device *dev, u32 index)
{
	int ret = 0;
	struct clk *clk;

	if (index >= vipx_clk_array_size) {
		vipx_err("index is invalid(%d >= %d)\n", index, vipx_clk_array_size);
		ret = -EINVAL;
		goto p_err;
	}

	clk = vipx_clk_array[index].clk;
	if (IS_ERR_OR_NULL(clk)) {
		vipx_err("clk is NULL(%d)\n", index);
		ret = -EINVAL;
		goto p_err;
	}

	clk_disable_unprepare(clk);

p_err:
	return ret;
}

static int vipx_exynos_clk_init(struct device *dev)
{
	int ret = 0;
	const char *name;
	struct clk *clk;
	u32 index;

	for (index = 0; index < vipx_clk_array_size; ++index) {
		name = vipx_clk_array[index].name;
		if (!name) {
			probe_err("name is NULL\n");
			ret = -EINVAL;
			break;
		}

		clk = clk_get(dev, name);
		if (IS_ERR_OR_NULL(clk)) {
			probe_err("%s clk is not found\n", name);
			ret = -EINVAL;
			break;
		}

		vipx_clk_array[index].clk = clk;
	}

	return ret;
}

int vipx_exynos_probe(struct vipx_exynos *exynos, struct device *dev,
	void **regs, void *ram0, void *ram1)
{
	int ret = 0;

	BUG_ON(!exynos);
	BUG_ON(!dev);

	exynos->regbase = regs;
	exynos->ram0base = ram0;
	exynos->ram1base = ram1;
	exynos->clk_ops = &vipx_clk_ops;
	exynos->ctl_ops = &vipx_ctl_ops;
	exynos->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(exynos->pinctrl)) {
		probe_err("devm_pinctrl_get is fail");
		ret = PTR_ERR(exynos->pinctrl);
		goto p_err;
	}

	ret = vipx_exynos_clk_init(dev);
	if (ret) {
		probe_err("vipx_exynos_clk_init is fail(%d)", ret);
		goto p_err;
	}

p_err:
	probe_info("%s():%d\n", __func__, ret);
	return ret;
}

void vipx_readl(void __iomem *base_addr, struct vipx_reg *reg, u32 *val)
{
	*val = readl(base_addr + reg->offset);

#ifdef DBG_HW_SFR
	vipx_info("[REG][%s][0x%04X], val(R):[0x%08X]\n", reg->name, reg->offset, *val);
#endif
}

void vipx_writel(void __iomem *base_addr, struct vipx_reg *reg, u32 val)
{
#ifdef DBG_HW_SFR
	vipx_info("[REG][%s][0x%04X], val(W):[0x%08X]\n", reg->name, reg->offset, val);
#endif

	writel(val, base_addr + reg->offset);
}

void vipx_readf(void __iomem *base_addr, struct vipx_reg *reg, struct vipx_field *field, u32 *val)
{
	*val = (readl(base_addr + reg->offset) >> (field->bit_start)) & ((1 << (field->bit_width)) - 1);

#ifdef DBG_HW_SFR
	vipx_info("[REG][%s][%s][0x%04X], val(R):[0x%08X]\n", reg->name, field->name, reg->offset, *val);
#endif
}

void vipx_writef(void __iomem *base_addr, struct vipx_reg *reg, struct vipx_field *field, u32 val)
{
	u32 mask, temp;

	mask = ((1 << field->bit_width) - 1);
	temp = readl(base_addr + reg->offset) & ~(mask << field->bit_start);
	temp |= (val & mask) << (field->bit_start);

#ifdef DBG_HW_SFR
	vipx_info("[REG][%s][%s][0x%04X], val(W):[0x%08X]\n", reg->name, field->name, reg->offset, val);
#endif

	writel(temp, base_addr + reg->offset);
}
