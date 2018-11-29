/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/pm_qos.h>
#include <linux/exynos_iovmm.h>
#include <asm/cacheflush.h>
#include <linux/io.h>

#include "vipx-config.h"
#include "vipx-device.h"
#include "vipx-system.h"

#define CAM_L0 690000
#define CAM_L1 680000
#define CAM_L2 670000
#define CAM_L3 660000
#define CAM_L4 650000

#define MIF_L0 2093000
#define MIF_L1 2002000
#define MIF_L2 1794000
#define MIF_L3 1539000
#define MIF_L4 1352000
#define MIF_L5 1014000
#define MIF_L6 845000
#define MIF_L7 676000

struct pm_qos_request exynos_vipx_qos_cam;
struct pm_qos_request exynos_vipx_qos_mem;

int vipx_system_probe(struct vipx_system *system, struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev;
	struct resource *res;
	void __iomem *iomem;
	int irq;

	BUG_ON(!system);
	BUG_ON(!pdev);

	dev = &pdev->dev;
	system->cam_qos = 0;
	system->mif_qos = 0;

	/* VIPX_CPU_SS1 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		probe_err("platform_get_resource(0) is fail(%p)", res);
		ret = PTR_ERR(res);
		goto p_err;
	}

	iomem =  devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(iomem)) {
		probe_err("devm_ioremap_resource(0) is fail(%p)", iomem);
		ret = PTR_ERR(iomem);
		goto p_err;
	}

	system->reg_ss[VIPX_REG_CPU_SS1] = iomem;
	system->reg_ss_size[VIPX_REG_CPU_SS1] = resource_size(res);
	probe_info("resource0 : %p\n", iomem);

	/* VIPX_CPU_SS2 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		probe_err("platform_get_resource(1) is fail(%p)", res);
		ret = PTR_ERR(res);
		goto p_err;
	}

	iomem =  devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(iomem)) {
		probe_err("devm_ioremap_resource(1) is fail(%p)", iomem);
		ret = PTR_ERR(iomem);
		goto p_err;
	}

	system->reg_ss[VIPX_REG_CPU_SS2] = iomem;
	system->reg_ss_size[VIPX_REG_CPU_SS2] = resource_size(res);
	probe_info("resource1 : %p\n", iomem);

	/* VIPX ITCM */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		probe_err("platform_get_resource(2) is fail(%p)", res);
		ret = PTR_ERR(res);
		goto p_err;
	}

	iomem =  devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(iomem)) {
		probe_err("devm_ioremap_resource(2) is fail(%p)", iomem);
		ret = PTR_ERR(iomem);
		goto p_err;
	}

	system->itcm = iomem;
	system->itcm_size = resource_size(res);
	probe_info("resource2 : %p\n", iomem);

	/* VIPX DTCM */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!res) {
		probe_err("platform_get_resource(3) is fail(%p)", res);
		ret = PTR_ERR(res);
		goto p_err;
	}

	iomem =  devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (IS_ERR_OR_NULL(iomem)) {
		probe_err("devm_ioremap_resource(3) is fail(%p)", iomem);
		ret = PTR_ERR(iomem);
		goto p_err;
	}

	system->dtcm = iomem;
	system->dtcm_size = resource_size(res);
	probe_info("resource3 : %p\n", iomem);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		probe_err("platform_get_irq(0) is fail(%d)", irq);
		ret = -EINVAL;
		goto p_err;
	}

	system->irq0 = irq;

	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		probe_err("platform_get_irq(1) is fail(%d)", irq);
		ret = -EINVAL;
		goto p_err;
	}

	system->irq1 = irq;

	ret = vipx_exynos_probe(&system->exynos, dev, system->reg_ss, system->itcm, system->dtcm);
	if (ret) {
		probe_err("vipx_exynos_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_memory_probe(&system->memory, dev);
	if (ret) {
		vipx_err("vipx_memory_probe is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_interface_probe(&system->interface, dev,
		system->reg_ss[VIPX_REG_CPU_SS1], system->reg_ss_size[VIPX_REG_CPU_SS1],
		system->irq0, system->irq1);
	if (ret) {
		vipx_err("vipx_interface_probe is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_binary_init(&system->binary, dev);
	if (ret) {
		vipx_err("vipx_binary_init is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	probe_info("%s():%d\n", __func__, ret);
	return ret;
}

int vipx_system_open(struct vipx_system *system)
{
	int ret = 0;

	ret = vipx_memory_open(&system->memory);
	if (ret) {
		vipx_err("vipx_memory_open is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_interface_open(&system->interface,
		(void *)system->memory.info.kvaddr_mbox, VIPX_MBOX_SIZE);
	if (ret) {
		vipx_err("vipx_interface_open is fail(%d)\n", ret);
		vipx_memory_close(&system->memory);
		goto p_err;
	}

p_err:
	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}

int vipx_system_close(struct vipx_system *system)
{
	int ret = 0;

#ifdef DUMP_DEBUG_LOG_REGION
	struct vipx_binary *binary;
	binary = &system->binary;

//	flush_all_cpu_caches();

	vipx_info("debug kva %p, heap kva %p\n", system->memory.info.kvaddr_debug, system->memory.info.kvaddr_heap);
	system->memory.info.pb_debug->ops->sync_for_cpu(system->memory.info.pb_debug, 0, system->memory.info.pb_debug->size, 0);
	if (!IS_ERR_OR_NULL(system->memory.info.kvaddr_debug)) {
		ret = vipx_binary_write(binary, VIPX_FW_PATH1, "vipx_log.bin", system->memory.info.kvaddr_debug, VIPX_DEBUG_SIZE);
		if (ret)
			vipx_err("vipx_binary_write is fail(%d)\n", ret);
	}

#endif

	ret = vipx_interface_close(&system->interface);
	if (ret)
		vipx_err("vipx_interface_close is fail(%d)\n", ret);

	ret = vipx_memory_close(&system->memory);
	if (ret)
		vipx_err("vipx_memory_close is fail(%d)\n", ret);

	return ret;
}

int vipx_system_resume(struct vipx_system *system, u32 mode)
{
	int ret = 0;
	struct vipx_exynos *exynos;
	struct vipx_memory *memory;

	BUG_ON(!system);

	exynos = &system->exynos;
	memory = &system->memory;

	ret = CLK_OP(exynos, clk_cfg);
	if (ret) {
		vipx_err("CLK_OP(clk_cfg) is fail(%d)\n", ret);
		goto p_err;
	}

#if defined(CONFIG_PM_DEVFREQ)
	pm_qos_add_request(&exynos_vipx_qos_cam, PM_QOS_CAM_THROUGHPUT, CAM_L0);
	pm_qos_add_request(&exynos_vipx_qos_mem, PM_QOS_BUS_THROUGHPUT, MIF_L0);
#endif

	ret = CLK_OP(exynos, clk_on);
	if (ret) {
		vipx_err("CLK_OP(clk_on) is fail(%d)\n", ret);
		goto p_err;
	}

#if defined(CONFIG_VIDEOBUF2_DMA_SG)
	system->memory.vipx_mem_ops->resume(system->memory.default_ctx);
#endif

	if (mode == VIPX_DEVICE_MODE_TEST)
		goto p_err;

	ret = vipx_system_fw_bootup(system);
	if (ret) {
		vipx_err("vipx_system_fw_bootup is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	return ret;
}

int vipx_system_suspend(struct vipx_system *system)
{
	int ret = 0;
	struct vipx_exynos *exynos;
	struct vipx_memory *memory;

	BUG_ON(!system);

	exynos = &system->exynos;
	memory = &system->memory;

#if defined(CONFIG_VIDEOBUF2_DMA_SG)
	system->memory.vipx_mem_ops->suspend(system->memory.default_ctx);
#endif

#if 0
	ret = CTL_OP(exynos, ctl_reset, 0);
	if (ret)
		vipx_err("CTL_OP(ctl_reset) is fail(%d)\n", ret);
#endif
	ret = CLK_OP(exynos, clk_off);
	if (ret)
		vipx_err("CLK_OP(clk_off) is fail(%d)\n", ret);

#if defined(CONFIG_PM_DEVFREQ)
	pm_qos_remove_request(&exynos_vipx_qos_cam);
	pm_qos_remove_request(&exynos_vipx_qos_mem);
#endif

	return ret;
}

int vipx_system_start(struct vipx_system *system)
{
	int ret = 0;

	ret = vipx_interface_start(&system->interface);
	if (ret)
		vipx_err("vipx_interface_start is fail(%d)\n", ret);

	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}

int vipx_system_stop(struct vipx_system *system)
{
	int ret = 0;

	ret = vipx_interface_stop(&system->interface);
	if (ret)
		vipx_err("vipx_interface_stop is fail(%d)\n", ret);

	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}

int vipx_system_fw_bootup(struct vipx_system *system)
{
	int ret = 0;
	struct vipx_exynos *exynos;
	struct vipx_memory *memory;
	struct vipx_binary *binary;

	struct {
		volatile u32 sign;
		volatile u32 mbox_addr;
		volatile u32 mbox_size;
		volatile u32 debug_addr;
		volatile u32 debug_size;
	} *tail_p;

	BUG_ON(!system);

//	execl("/system/bin/cp", "/system/bin/cp", "/d/pm_qos/cam_throughput", "/data", (char *)0);
//	execl("/system/bin/cp", "/system/bin/cp", "/d/pm_qos/bus_throughput", "/data", (char *)0);
	exynos = &system->exynos;
	memory = &system->memory;
	binary = &system->binary;

	ret = CTL_OP(exynos, ctl_reset, 0);
	if (ret) {
		vipx_err("CTL_OP(ctl_reset) is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_binary_read(binary, VIPX_FW_PATH1, VIPX_FW_DRAM_NAME,
			(void *)system->memory.info.kvaddr_fw, VIPX_CM7_DRAM_BIN_SIZE);
	if (ret) {
		ret = vipx_binary_read(binary, VIPX_FW_PATH2, VIPX_FW_DRAM_NAME,
				(void *)system->memory.info.kvaddr_fw, VIPX_CM7_DRAM_BIN_SIZE);
		if (ret) {
			vipx_err("vipx_dram_binary_load is fail(%d)\n", ret);
			goto p_err;
		}
	}

	ret = vipx_binary_read(binary, VIPX_FW_PATH1, VIPX_FW_DTCM_NAME,
			(void *)system->dtcm, system->dtcm_size);
	if (ret) {
		ret = vipx_binary_read(binary, VIPX_FW_PATH2, VIPX_FW_DTCM_NAME,
				(void *)system->dtcm, system->dtcm_size - SZ_1K);
		if (ret) {
			vipx_err("vipx_dtcm_binary_load is fail(%d)\n", ret);
			goto p_err;
		}
	}

	tail_p = (void *)(system->dtcm + 0x3FD0);
	tail_p->sign = 0xABCDEFAB;
	tail_p->mbox_addr = system->memory.info.dvaddr_mbox;
	tail_p->mbox_size = VIPX_MBOX_SIZE;
	tail_p->debug_addr = system->memory.info.dvaddr_debug;
	tail_p->debug_size = VIPX_DEBUG_SIZE;

    vipx_info("host_shared : %p, mbox_addr : %x, mbox_size : %x, debug_addr : %x, debug_size : %x\n",
        tail_p, tail_p->mbox_addr, tail_p->mbox_size, tail_p->debug_addr, tail_p->debug_size);

	ret = vipx_binary_read(binary, VIPX_FW_PATH1, VIPX_FW_ITCM_NAME,
			(void *)system->itcm, system->itcm_size);
	if (ret) {
		ret = vipx_binary_read(binary, VIPX_FW_PATH2, VIPX_FW_ITCM_NAME,
				(void *)system->itcm, system->itcm_size);
		if (ret) {
			vipx_err("vipx_itcm_binary_load is fail(%d)\n", ret);
			goto p_err;
		}
	}

//	flush_all_cpu_caches();

	ret = CTL_OP(exynos, ctl_start, 0);
	if (ret) {
		vipx_err("CTL_OP(ctl_start) is fail(%d)\n", ret);
		goto p_err;
	}
#if 0	// change code position to vipx_vertex_streamon in vipx-vertex.c before streamon execution
	ret = vipx_hw_wait_bootup(&system->interface);
	if (ret) {
		vipx_err("vipx_hw_wait_bootup is fail(%d)\n", ret);
		goto p_err;
	}
#endif

p_err:
	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}
