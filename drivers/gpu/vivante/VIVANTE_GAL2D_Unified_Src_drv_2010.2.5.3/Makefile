OS_KERNEL_DIR   := hal/os/linux/kernel
ARCH_KERNEL_DIR := arch/XAQ2/hal/kernel
HAL_KERNEL_DIR  := hal/kernel

VIV_DRIVER_PATH := drivers/gpu/vivante/VIVANTE_GAL2D_Unified_Src_drv_2010.2.5.3

# GC200 galcore config
EXTRA_CFLAGS += -DLINUX -DDRIVER -DENUM_WORKAROUND=0 -DDBG=0 -DNO_DMA_COHERENT -DENABLE_ARM_L2_CACHE=0 -DUSE_PLATFORM_DRIVER=1 -DVIVANTE_PROFILER=0 -DANDROID=0 -DENABLE_GPU_CLOCK_BY_DRIVER=0 -DUSE_NEW_LINUX_SIGNAL=0 -DNO_USER_DIRECT_ACCESS_FROM_KERNEL=0  -I$(VIV_DRIVER_PATH)/hal/inc -I$(VIV_DRIVER_PATH)/hal/kernel -I$(VIV_DRIVER_PATH)/arch/XAQ2/hal/kernel -I$(VIV_DRIVER_PATH)/hal/user/

$(shell mkdir -p $(VIV_DRIVER_PATH)/$(OS_KERNEL_DIR))
$(shell mkdir -p $(VIV_DRIVER_PATH)/$(ARCH_KERNEL_DIR))
$(shell mkdir -p $(VIV_DRIVER_PATH)/$(HAL_KERNEL_DIR))

obj-y += $(OS_KERNEL_DIR)/gc_hal_kernel_linux.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_debug.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_device.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_os.o \
        $(OS_KERNEL_DIR)/gc_hal_kernel_driver.o


obj-y += $(HAL_KERNEL_DIR)/gc_hal_kernel_mmu.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_heap.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_video_memory.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_command.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel_event.o \
        $(HAL_KERNEL_DIR)/gc_hal_kernel.o

obj-y += $(ARCH_KERNEL_DIR)/gc_hal_kernel_hardware.o
