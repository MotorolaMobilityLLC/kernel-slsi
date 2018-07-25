/*
 * Samsung Exynos SoC series VIPx driver
 *
 * Copyright (c) 2017 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/exynos_iovmm.h>
#include <linux/pm_runtime.h>
#include <linux/memblock.h>
#include <linux/of_reserved_mem.h>

#include "vipx-config.h"
#include "vipx-device.h"
#include "vipx-graph.h"
#include "vipx-vertex.h"
#include "vipx-graphmgr.h"

static struct reserved_mem *vipx_fw_code_rmem;
void *vipx_fw_code_rmem_base;

static int vipx_device_runtime_suspend(struct device *dev);
static int vipx_device_runtime_resume(struct device *dev);

static int __init vipx_fw_code_rmem_setup(struct reserved_mem *rmem)
{
	pr_info("%s: base=%pa, size=%pa\n", __func__, &rmem->base, &rmem->size);

	vipx_fw_code_rmem = rmem;

	return 0;
}

RESERVEDMEM_OF_DECLARE(vipx_fw_code_rmem, "exynos,vipx_fw_code_rmem", vipx_fw_code_rmem_setup);

void __vipx_fault_handler(struct vipx_device *device) {
#if 0
	struct vipx_system *system;
	struct vipx_memory *memory;
	struct vipx_graphmgr *graphmgr;
	struct vipx_graph *graph;
	struct vipxo_pu *pu, *temp;
	struct vb_container *container;
	struct vb_buffer *buffer;
	u32 i, j, k;

	system = &device->system;
	memory = &system->memory;
	graphmgr = &device->graphmgr;

	/* 1. Internal Memory Infomation */
	vipx_info("internal memory : 0x%llX\n", memory->info.dvaddr);

	/* 2. Buffer Memroy Information */
	for (i = 0; i < VIPX_MAX_GRAPH; ++i) {
		graph = graphmgr->graph[i];
		if (!graph)
			continue;

		vipx_info("=================================================\n");
		vipx_info("GRAPH : %d(%d)\n", graph->id, graph->uid);
		vipx_info("INPUT--------------------------------------------\n");
		list_for_each_entry_safe(pu, temp, &graph->inleaf_list, gleaf_entry) {
			vipx_info("PU : %d\n", pu->id);
			vipx_info("-------------------------------------------------\n");
			for (j = 0; j < VIPX_MAX_BUFFER; ++j) {
				container = pu->container[j];
				if (!container)
					continue;

				if (j == 0) {
					vipx_info("TYPE : %d\n", container->type);
					vipx_info("RESOLUTION : %d x %d\n", container->format->width, container->format->height);
					vipx_info("SIZE : %d\n", container->format->size[container->format->plane]);
					vipx_info("-------------------------------------------------\n");
				}

				for (k = 0; k < container->count; ++k) {
					buffer = &container->buffers[k];
					vipx_info("[%d][%d]DVADDR : %llx\n", j, k, buffer->dvaddr);
					vipx_info("[%d][%d]KVADDR : %p\n", j, k, buffer->kvaddr);
				}

				vipx_info("-------------------------------------------------\n");
			}
		}

		vipx_info("OUTPUT-------------------------------------------\n");
		list_for_each_entry_safe(pu, temp, &graph->otleaf_list, gleaf_entry) {
			vipx_info("PU : %d\n", pu->id);
			vipx_info("-------------------------------------------------\n");
			for (j = 0; j < VIPX_MAX_BUFFER; ++j) {
				container = pu->container[j];
				if (!container)
					continue;

				if (j == 0) {
					vipx_info("TYPE : %d\n", container->type);
					vipx_info("RESOLUTION : %d x %d\n", container->format->width, container->format->height);
					vipx_info("SIZE : %d\n", container->format->size[container->format->plane]);
					vipx_info("-------------------------------------------------\n");
				}

				for (k = 0; k < container->count; ++k) {
					buffer = &container->buffers[k];
					vipx_info("[%d][%d]DVADDR : %llx\n", j, k, buffer->dvaddr);
					vipx_info("[%d][%d]KVADDR : %p\n", j, k, buffer->kvaddr);
				}

				vipx_info("-------------------------------------------------\n");
			}
		}

		vipx_graph_print(graph);
	}
#endif
}

static int __attribute__((unused)) vipx_fault_handler(struct iommu_domain *domain,
	struct device *dev,
	unsigned long fault_addr,
	int fault_flag,
	void *token)
{
	struct vipx_device *device;

	pr_err("<VIPX FAULT HANDLER>\n");
	pr_err("Device virtual(0x%X) is invalid access\n", (u32)fault_addr);

	device = dev_get_drvdata(dev);

	__vipx_fault_handler(device);

	return -EINVAL;
}

static int __vipx_device_start(struct vipx_device *device)
{
	int ret = 0;

	if (test_bit(VIPX_DEVICE_STATE_START, &device->state)) {
		vipx_err("already started\n");
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_system_start(&device->system);
	if (ret) {
		vipx_err("vipx_system_start is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_debug_start(&device->debug);
	if (ret) {
		vipx_err("vipx_debug_start is fail(%d)\n", ret);
		goto p_err;
	}

	set_bit(VIPX_DEVICE_STATE_START, &device->state);

p_err:
	return ret;
}

static int __vipx_device_stop(struct vipx_device *device)
{
	int ret = 0;

	if (!test_bit(VIPX_DEVICE_STATE_START, &device->state))
		goto p_err;

	ret = vipx_debug_stop(&device->debug);
	if (ret)
		vipx_err("vipx_debug_stop is fail(%d)\n", ret);

	ret = vipx_system_stop(&device->system);
	if (ret)
		vipx_err("vipx_system_stop is fail(%d)\n", ret);

	clear_bit(VIPX_DEVICE_STATE_START, &device->state);

p_err:
	return ret;
}

static int __vipx_device_power_on(struct vipx_device *device)
{
	int ret = 0;

	ret = pm_runtime_get_sync(device->dev);
	if (ret)
		vipx_err("runtime resume is fail(%d)", ret);

	return ret;
}

static int __vipx_device_power_off(struct vipx_device *device)
{
	int ret = 0;

	ret = pm_runtime_put_sync(device->dev);
	if (ret)
		vipx_err("runtime resume is fail(%d)", ret);

	return ret;
}

static int vipx_device_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev;
	struct vipx_device *device;

	BUG_ON(!pdev);

    dma_set_mask(&pdev->dev, DMA_BIT_MASK(36));

	dev = &pdev->dev;

	device = devm_kzalloc(dev, sizeof(struct vipx_device), GFP_KERNEL);
	if (!device) {
		probe_err("device is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	ret = vipx_system_probe(&device->system, pdev);
	if (ret) {
		probe_err("vipx_system_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_vertex_probe(&device->vertex, dev);
	if (ret) {
		probe_err("vipx_vertex_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_graphmgr_probe(&device->graphmgr);
	if (ret) {
		probe_err("vipx_graphmgr_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_debug_probe(&device->debug, &device->graphmgr, &device->system);
	if (ret) {
		probe_err("vipx_debug_probe is fail(%d)\n", ret);
		ret = -EINVAL;
		goto p_err;
	}

	iovmm_set_fault_handler(dev, vipx_fault_handler, NULL);
	pm_runtime_enable(dev);

	device->dev = dev;
	device->mode = VIPX_DEVICE_MODE_NORMAL;
	clear_bit(VIPX_DEVICE_STATE_OPEN, &device->state);
	clear_bit(VIPX_DEVICE_STATE_START, &device->state);
	dev_set_drvdata(dev, device);

	vipx_fw_code_rmem_base = phys_to_virt(vipx_fw_code_rmem->base);

p_err:

	probe_info("%s():%d\n", __func__, ret);
	return ret;
}

int vipx_device_open(struct vipx_device *device)
{
	int ret = 0;

	BUG_ON(!device);

	if (test_bit(VIPX_DEVICE_STATE_OPEN, &device->state)) {
		vipx_err("device is already opened\n");
		ret = -EINVAL;
		goto p_err;
	}

	ret = vipx_system_open(&device->system);
	if (ret) {
		vipx_err("vipx_system_open is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_debug_open(&device->debug);
	if (ret) {
		vipx_err("vipx_debug_open is fail(%d)\n", ret);
		goto p_err;
	}

	ret = vipx_graphmgr_open(&device->graphmgr);
	if (ret) {
		vipx_err("vipx_graphmgr_open is fail(%d)\n", ret);
		goto p_err;
	}

	ret = __vipx_device_power_on(device);
	if (ret) {
		vipx_err("__vipx_device_power_on is fail(%d)\n", ret);
		goto p_err;
	}

	set_bit(VIPX_DEVICE_STATE_OPEN, &device->state);

p_err:
	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}

int vipx_device_close(struct vipx_device *device)
{
	int ret = 0;

	BUG_ON(!device);

	if (!test_bit(VIPX_DEVICE_STATE_OPEN, &device->state)) {
		vipx_err("device is already closed\n");
		ret = -EINVAL;
		goto p_err;
	}

	ret = __vipx_device_stop(device);
	if (ret)
		vipx_err("__vipx_device_stop is fail(%d)\n", ret);

	ret = vipx_graphmgr_close(&device->graphmgr);
	if (ret)
		vipx_err("vipx_graphmgr_close is fail(%d)\n", ret);

	ret = vipx_system_close(&device->system);
	if (ret)
		vipx_err("vipx_system_close is fail(%d)\n", ret);

	ret = vipx_debug_close(&device->debug);
	if (ret)
		vipx_err("vipx_debug_close is fail(%d)\n", ret);

	ret = __vipx_device_power_off(device);
	if (ret)
		vipx_err("__vipx_device_power_off is fail(%d)\n", ret);

	clear_bit(VIPX_DEVICE_STATE_OPEN, &device->state);

p_err:
	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}

int vipx_device_start(struct vipx_device *device)
{
	int ret = 0;

	BUG_ON(!device);

	ret = __vipx_device_start(device);
	if (ret)
		vipx_err("__vipx_device_start is fail(%d)\n", ret);

	vipx_info("%s():%d\n", __func__, ret);

	return ret;
}

int vipx_device_stop(struct vipx_device *device)
{
	int ret = 0;

	BUG_ON(!device);

	ret = __vipx_device_stop(device);
	if (ret)
		vipx_err("__vipx_device_stop is fail(%d)\n", ret);

	vipx_info("%s():%d\n", __func__, ret);

	return ret;
}

static int vipx_device_remove(struct platform_device *pdev)
{
	return 0;
}

static int vipx_device_suspend(struct device *dev)
{
	return 0;
}

static int vipx_device_resume(struct device *dev)
{
	return 0;
}

static int vipx_device_runtime_suspend(struct device *dev)
{
	int ret = 0;
	struct vipx_device *device;

	device = dev_get_drvdata(dev);

	ret = vipx_system_suspend(&device->system);
	if (ret)
		vipx_err("vipx_system_suspend is fail(%d)\n", ret);

	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}

static int vipx_device_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct vipx_device *device;

	device = dev_get_drvdata(dev);

	ret = vipx_system_resume(&device->system, device->mode);
	if (ret) {
		vipx_err("vipx_system_resume is fail(%d)\n", ret);
		goto p_err;
	}

p_err:
	vipx_info("%s():%d\n", __func__, ret);
	return ret;
}

static const struct dev_pm_ops vipx_pm_ops = {
	.suspend		= vipx_device_suspend,
	.resume			= vipx_device_resume,
	.runtime_suspend	= vipx_device_runtime_suspend,
	.runtime_resume		= vipx_device_runtime_resume,
};

static const struct of_device_id exynos_vipx_match[] = {
	{
		.compatible = "samsung,exynos-vipx",
	},
	{}
};
MODULE_DEVICE_TABLE(of, exynos_vipx_match);

static struct platform_driver vipx_driver = {
	.probe		= vipx_device_probe,
	.remove		= vipx_device_remove,
	.driver = {
		.name	= "exynos-vipx",
		.owner	= THIS_MODULE,
		.pm	= &vipx_pm_ops,
		.of_match_table = of_match_ptr(exynos_vipx_match)
	}
};

static int __init vipx_device_init(void)
{
	int ret = platform_driver_register(&vipx_driver);
	if (ret)
		probe_err("platform_driver_register is fail():%d\n", ret);

	probe_info("vipx device init is loaded");

	return ret;
}
late_initcall(vipx_device_init);

static void __exit vipx_device_exit(void)
{
	platform_driver_unregister(&vipx_driver);
}
module_exit(vipx_device_exit);

MODULE_AUTHOR("Scott Choi<hyeon.choi@samsung.com>");
MODULE_DESCRIPTION("Exynos VIPx driver");
MODULE_LICENSE("GPL");
