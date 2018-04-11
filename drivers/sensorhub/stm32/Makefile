#
# Makefile for the sensor drivers.
#

# Each configuration option enables a list of files.

subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat


obj-$(CONFIG_SENSORS_SSP)		+= sensors_core.o ssp_dev.o ssp_spi.o ssp_data.o ssp_sysfs.o \
						ssp_iio.o ssp_firmware.o ssp_debug.o ssp_sensorhub.o \
						factory/
