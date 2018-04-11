#
# Makefile for the sensor drivers.
#

# Each configuration option enables a list of files.

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/mt6757

ifeq ($(CONFIG_SENSORS_SSP_STM32_MTK), y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/magnetometer/inc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/gyroscope/inc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/alsps/inc
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/accelerometer/inc
endif

obj-$(CONFIG_SENSORS_SSP) += sensors_core.o ssp_dev.o ssp_spi.o ssp_data.o ssp_sysfs.o \
						ssp_firmware.o ssp_debug.o ssp_sensorhub.o \
						factory/
ifeq ($(CONFIG_SENSORS_SSP_STM32_MTK), y)
obj-$(CONFIG_SENSORS_SSP) += ssp_mtk_sensors.o
else
obj-$(CONFIG_SENSORS_SSP) += ssp_iio.o
endif
