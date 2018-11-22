########################################################################
# Copyright (c) 2018 Samsung Electronics Co., Ltd. All rights reserved #
########################################################################

# Needed since this subdir is symlinked in the main Kernel tree
# without this our samsung subdir is NOT cleaned.
clean-files := *.o *.ko

ccflags-y += $(CONFIG_SAMSUNG_MAXWELL_EXTRA)

# No such thing as ccflags-m, so ifeq(..., m)/else
# to correctly handle all options (y,m,n)
ifeq ($(CONFIG_SCSC_DEBUG), m)
ccflags-y += -DCONFIG_SCSC_PRINTK
else
ccflags-$(CONFIG_SCSC_LOGRING) += -DCONFIG_SCSC_PRINTK
endif

ifneq ($(CONFIG_SCSC_STATIC_RING), n)
ccflags-y += -DCONFIG_SCSC_STATIC_RING_SIZE=$(CONFIG_SCSC_STATIC_RING_SIZE)
endif

# Debug
obj-$(CONFIG_SCSC_LOGRING) += scsc_logring.o
scsc_logring-$(CONFIG_SCSC_LOGRING) += \
	scsc_logring_main.o \
	scsc_logring_ring.o \
	scsc_logring_debugfs.o

# Log collection
obj-$(CONFIG_SCSC_LOG_COLLECTION) += scsc_log_collection.o
scsc_log_collection-$(CONFIG_SCSC_LOG_COLLECTION) += \
	scsc_log_collector.o \
	scsc_log_collector_proc.o \
	scsc_log_collector_mmap.o \

# WiFi logger
obj-$(CONFIG_SCSC_WIFILOGGER) += scsc_wifilogger.o
scsc_wifilogger-$(CONFIG_SCSC_WIFILOGGER) += \
	scsc_wifilogger_api.o \
	scsc_wifilogger_internal.o \
	scsc_wifilogger_core.o \
	scsc_wifilogger_module.o \
	scsc_wifilogger_ring_connectivity.o \
	scsc_wifilogger_ring_wakelock.o \
	scsc_wifilogger_ring_pktfate.o

scsc_wifilogger-$(CONFIG_SCSC_WIFILOGGER_DEBUGFS) += scsc_wifilogger_debugfs.o

scsc_wifilogger-$(CONFIG_SCSC_WIFILOGGER_TEST) += scsc_wifilogger_ring_test.o

# PCI-E MIF
obj-$(CONFIG_SCSC_PCIE) += scsc_pcie_mif.o
scsc_pcie_mif-$(CONFIG_SCSC_PCIE) += \
	pcie_mif_module.o \
	pcie_mif.o \
	pcie_proc.o \
	pcie_mbox.o \
	pcie_mbox_intgen.o

# Platform MIF
obj-$(CONFIG_SCSC_PLATFORM) += scsc_platform_mif.o
scsc_platform_mif-$(CONFIG_SCSC_PLATFORM) += \
	platform_mif_module.o \
	platform_mif.o

# Maxwell manager
obj-$(CONFIG_SCSC_CORE_CM) += scsc_mx.o
scsc_mx-$(CONFIG_SCSC_CORE_CM) += \
	mxlog_transport.o \
	fw_panic_record.o \
	panicmon.o \
	suspendmon.o \
	mxlog.o \
	mxproc.o \
	scsc_service.o \
	scsc_mx_module.o \
	scsc_mx_impl.o \
	mxman.o \
	miframman.o \
	mifmboxman.o \
	mifproc.o \
	mifintrbit.o \
	fwhdr.o \
	fwimage.o \
	cpacket_buffer.o \
	mifstream.o \
	mxmgmt_transport.o \
	gdb_transport.o \
	scsc_lerna.o \
	mxfwconfig.o \
	mx140_file.o

# Maxwell logger
scsc_mx-$(CONFIG_SCSC_MXLOGGER) += mxlogger.o

# SMAPPER
scsc_mx-$(CONFIG_SCSC_SMAPPER) += mifsmapper.o

# WLBT daemon
scsc_mx-$(CONFIG_SCSC_WLBTD) += scsc_wlbtd.o

# QOS
scsc_mx-$(CONFIG_SCSC_QOS) += mifqos.o

# Null service (test)
obj-$(CONFIG_SCSC_CM_MX_CLIENT_TEST) += mx_client_test.o
mx_client_test-$(CONFIG_SCSC_CM_MX_CLIENT_TEST) += client_test.o

# Memory map
obj-$(CONFIG_SCSC_MMAP) += scsc_mmap.o
scsc_mmap-$(CONFIG_SCSC_MMAP) += mx_mmap.o

# 20 MHz clock test
obj-$(CONFIG_SCSC_CLK20MHZ) += scsc_mx140_clk.o
scsc_mx140_clk-$(CONFIG_SCSC_CLK20MHZ) += mx140_clk.o
ifeq ($(CONFIG_SCSC_CLK20MHZ_TEST),y)
scsc_mx140_clk-$(CONFIG_SCSC_CLK20MHZ) += mx140_clk_test.o
endif

# FM LDO test
obj-$(CONFIG_SCSC_FM) += scsc_mx250_fm.o
scsc_mx250_fm-$(CONFIG_SCSC_FM) += mx250_fm.o
ifeq ($(CONFIG_SCSC_FM_TEST),y)
scsc_mx250_fm-$(CONFIG_SCSC_FM) += mx250_fm_test.o
endif

# Debug sampler
obj-$(CONFIG_SCSC_DBG_SAMPLER) += scsc_dbg_sampler.o
scsc_dbg_sampler-$(CONFIG_SCSC_DBG_SAMPLER) += mx_dbg_sampler.o

# The below section is only for DTE purpose. It is controlled by the flag SCSC_DTE_BUILD which is set only by the
# DTE top level Makefile. Hence, no risk.
# In the below include dir the KERNEL_DIR comes from the DTE top level makefile.
ifeq ($(SCSC_DTE_BUILD),y)
obj-m := scsc_mx.o
obj-m += mx_client_test.o
mx_client_test-y += client_test.o

ccflags-y +=  -I$(KERNEL_DIR)/include/scsc/
ccflags-y +=  -I$(KERNEL_DIR)/include
ccflags-y +=  -I$(KERNEL_DIR)/drivers/misc/samsung/scsc
ccflags-y +=  -I$(KERNEL_DIR)/drivers/net/wireless/scsc

endif
