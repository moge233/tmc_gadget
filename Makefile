
KERNEL_DIR ?= $(HOME)/keithley/ti-linux/linux-6.6.11

USB_TMC_FUNC = f_tmc
obj-m += $(USB_TMC_FUNC).o

all:
	make -C $(KERNEL_DIR) ARCH=arm64 CROSS_COMPILE=aarch64-buildroot-linux-gnu- M=$(PWD) modules

	
.PHONY: clean
clean:
	make -C $(KERNEL_DIR) ARCH=arm64 CROSS_COMPILE=aarch64-buildroot-linux-gnu- M=$(PWD) clean