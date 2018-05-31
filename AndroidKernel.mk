#Android makefile to build kernel as a part of Android Build

ifeq ($(KERNEL_DEFCONFIG),)
$(error KERNEL_DEFCONFIG must be set as environment variable)
endif

ifeq ($(INSTALLED_KERNEL_TARGET),)
INSTALLED_KERNEL_TARGET := $(PRODUCT_OUT)/kernel
INSTALLED_DTBOIMAGE_TARGET := $(PRODUCT_OUT)/dtbo.img
BOARD_PREBUILT_DTBOIMAGE := $(PRODUCT_OUT)/dtbo.img
INSTALLED_DTB_TARGET := $(PRODUCT_OUT)/dtb.img
endif

TARGET_KERNEL_ARCH := $(strip $(TARGET_KERNEL_ARCH))
ifeq ($(TARGET_KERNEL_ARCH),)
KERNEL_ARCH := arm64
else
KERNEL_ARCH := $(TARGET_KERNEL_ARCH)
endif

ifeq ($(CROSS_COMPILE),)
KERNEL_CROSS_COMPILE := aarch64-linux-android-
else
KERNEL_CROSS_COMPILE := $(CROSS_COMPILE)
endif

ifeq ($(CLANG_TRIPLE),)
CLANG_TRIPLE := aarch64-linux-gnu-
else
CLANG_TRIPLE := $(CLANG_TRIPLE)
endif

ifeq ($(TARGET_PREBUILT_KERNEL),)

TARGET_KERNEL_SOURCE := kernel/$(TARGET_KERNEL)
KERNEL_CONFIG := $(TARGET_KERNEL_SOURCE)/.config
KERNEL_BOOT := $(TARGET_KERNEL_SOURCE)/arch/$(KERNEL_ARCH)/boot
KERNEL_BIN := $(KERNEL_BOOT)/Image
KERNEL_DTB_DIR := $(KERNEL_BOOT)/dts/exynos
KERNEL_DTB := $(KERNEL_DTB_DIR)/exynos9610.dtb
KERNEL_DTBO_CFG := $(KERNEL_DTB_DIR)/exynos9610_dtboimg.cfg
MKDTIMG := $(HOST_OUT_EXECUTABLES)/mkdtimg

ifeq ($(KERNEL_DEFCONFIG),)
$(error Kernel configuration not defined, cannot build kernel)
else

ifeq ($(N_KERNEL_BUILD_THREAD),)
N_KERNEL_BUILD_THREAD := 1
endif

TARGET_PREBUILT_KERNEL := $(KERNEL_BIN)

.PHONY: remove-bins
remove-bin:
	$(hide) echo "Clean Up prebuilts"
	rm -f $(KERNEL_CONFIG)
	rm -f $(KERNEL_BIN)
	rm -f $(KERNEL_DTB)
	rm -f $(INSTALLED_KERNEL_TARGET)
	rm -f $(INSTALLED_DTBOIMAGE_TARGET)
	rm -f $(INSTALLED_DTB_TARGET)

.PHONY: kernel
kernel: $(KERNEL_BIN)

.PHONY: kernel-distclean
kernel-distclean:
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) distclean

$(KERNEL_CONFIG): remove-bins
	$(hide) echo "make $(KERNEL_DEFCONFIG)"
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KERNEL_DEFCONFIG)

$(KERNEL_BIN): $(KERNEL_CONFIG)
	$(hide) echo "Building kernel..."
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) CLANG_TRIPLE=$(CLANG_TRIPLE) CC=clang -j$(N_KERNEL_BUILD_THREAD)

# mkdtimg cfg_create exynos9610_dtbo.img arch/arm64/boot/dts/exynos/exynos9610_dtboimg.cfg
$(INSTALLED_KERNEL_TARGET): $(KERNEL_BIN) $(MKDTIMG)
	cp $(KERNEL_BIN) $(INSTALLED_KERNEL_TARGET)
	cp $(KERNEL_DTB) $(INSTALLED_DTB_TARGET)
	$(hide) echo "Building DTBOIMAGE..."
	ln -sf $(TARGET_KERNEL_SOURCE)/arch
	$(MKDTIMG) cfg_create $(INSTALLED_DTBOIMAGE_TARGET) $(KERNEL_DTBO_CFG)
	rm -f arch

endif #TARGET_PREBUILT_KERNEL
endif #KERNEL_DEFCONFIG
