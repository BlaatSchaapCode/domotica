################################################################################
# Set the MCU
################################################################################
MCU ?= STM32F103xB

################################################################################
# Set Output Filename 
################################################################################
TARGET = usb_dongle

################################################################################
# Set the root directory of the ucdev enviorement 
################################################################################
UCDEV_ROOT=../ucdev
U8G2_DIR = $(UCDEV_ROOT)/ext/u8g2/csrc
BSSENSOR_DIR = $(UCDEV_ROOT)/lib/bssensor
BSRADIO_DIR = $(UCDEV_ROOT)/lib/bsradio
BSPROT_DIR =  $(UCDEV_ROOT)/lib/bsprot
BSRADIO_DIR = $(UCDEV_ROOT)/lib/bsradio

C_SOURCES += wireless_dongle.c
C_SOURCES += timer_stm32.c
C_SOURCES += usb.c
C_SOURCES += serial.c

# USB
#C_SOURCES += $(LIBHALGLUE_ROOT)/common/serialnumber.c
C_SOURCES += $(LIBHLUSBD_ROOT)/drivers/usbd_stm.c
C_SOURCES += $(LIBHLUSBD_ROOT)/usbd.c
C_SOURCES += $(LIBHLUSBD_ROOT)/usbd_descriptors.c
C_SOURCES += $(LIBHLUSBD_ROOT)/ConvertUTF/ConvertUTF.c
C_INCLUDES += $(LIBHLUSBD_ROOT)/
C_INCLUDES += $(LIBHLUSBD_ROOT)/drivers/
C_INCLUDES += $(LIBHLUSBD_ROOT)/ConvertUTF

C_SOURCES += $(BSPROT_DIR)/protocol.c

C_SOURCES += $(LIBHALGLUE_ROOT)/common/arm_cpuid.c
C_SOURCES += $(LIBHALGLUE_ROOT)/common/nosysfix.c
C_SOURCES += $(LIBHALGLUE_ROOT)/common/bshal_delay.c
C_SOURCES += $(LIBHALGLUE_ROOT)/bshal/bshal_i2cm.c
C_SOURCES += $(LIBHALGLUE_ROOT)/bshal/bshal_spim.c

C_SOURCES += $(BSRADIO_DIR)/bsradio.c
C_SOURCES += $(BSRADIO_DIR)/sxv1.c
C_SOURCES += $(BSRADIO_DIR)/si4x3x.c
C_SOURCES += $(BSRADIO_DIR)/si4x6x.c
C_SOURCES += $(BSRADIO_DIR)/magic.c

C_INCLUDES += $(BSRADIO_DIR)
C_INCLUDES += $(BSPROT_DIR)

C_INCLUDES += $(LIBHALGLUE_ROOT)/bshal/extlib/
C_INCLUDES += $(LIBHALGLUE_ROOT)/common/
C_INCLUDES += $(LIBHALGLUE_ROOT)/bshal/
C_INCLUDES += $(LIBHALGLUE_ROOT)/bshal/stm32
C_INCLUDES += $(LIBHALGLUE_ROOT)/hal/

C_INCLUDES += $(UCDEV_ROOT)/lib/spi_flash
C_SOURCES += $(UCDEV_ROOT)/lib/spi_flash/spi_flash.c

LDFLAGS +=  -L$(UCDEV_ROOT)/build/staticlib/rtt/debug/arm_m3  -lrtt
C_SOURCES +=  $(UCDEV_ROOT)/ext/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.c
C_INCLUDES += $(UCDEV_ROOT)/ext/RTT//Syscalls/
C_INCLUDES += $(UCDEV_ROOT)/ext/RTT//RTT/
C_INCLUDES += $(UCDEV_ROOT)/ext/RTT//Config/


C_SOURCES += $(LIBHALGLUE_ROOT)/hal/stm32f1/clocksetup.c
C_SOURCES += $(LIBHALGLUE_ROOT)/bshal/stm32/bshal_i2cm_stm32f1.c
C_SOURCES += $(LIBHALGLUE_ROOT)/bshal/stm32/bshal_spim_stm32f1.c
C_SOURCES += $(LIBHALGLUE_ROOT)/bshal/stm32/bshal_gpio_stm32.c
C_SOURCES += $(LIBHALGLUE_ROOT)/bshal/stm32/bshal_i2cm_stm32.c




# link script
LDSCRIPT = $(shell tr '[:upper:]' '[:lower:]' <<< $(MCU)).ld

MK_DIR=$(UCDEV_ROOT)/build/make
include $(MK_DIR)/all.mk
