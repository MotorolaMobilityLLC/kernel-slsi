# Makefile of modem_if

obj-y += modem_main.o \
	modem_io_device.o \
	modem_net_flowcontrol_device.o \
	modem_utils.o \
	modem_dump.o \
	modem_notifier.o

obj-$(CONFIG_LINK_DEVICE_SHMEM)	+= modem_link_device_shmem.o \
				modem_link_device_memory.o

obj-$(CONFIG_MODEMCTRL_MODAP_S3XXAP) += modem_modemctl_device_sh333ap.o

obj-$(CONFIG_UART_SWITCH) += uart_switch.o