
KERNEL_DIR ?= $(HOME)/keithley/ti-linux/linux-6.6.11

USB_TMC_FUNC = f_tmc
obj-m += $(USB_TMC_FUNC).o

all:
	make -C $(KERNEL_DIR) ARCH=arm64 CROSS_COMPILE=aarch64-buildroot-linux-gnu- M=$(PWD) modules && \
	mv -f *.ko		kobj && \
	mv -f *.o		obj  && \
	mv -f *.mod		mod  && \
	mv -f *.mod.c	mod  && \
	mv -f *.symvers symvers && \
	mv -f *.order   order

	
.PHONY: clean
clean:
	make -C $(KERNEL_DIR) ARCH=arm64 CROSS_COMPILE=aarch64-buildroot-linux-gnu- M=$(PWD) clean