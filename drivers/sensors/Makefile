#
# Makefile for the sensors drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_SENSORS_CORE)          += sensors_core.o

# accelerometer_sensor
obj-$(CONFIG_ST_LSM6DS3)            += st_lsm6ds3.o
st_lsm6ds3-objs := st_lsm6ds3_core.o
obj-$(CONFIG_ST_LSM6DS3_I2C)        += st_lsm6ds3_i2c.o
obj-$(CONFIG_ST_LSM6DS3_SPI)        += st_lsm6ds3_spi.o
obj-$(CONFIG_SENSORS_LSM6DSL)            += lsm6dsl_core.o
obj-$(CONFIG_SENSORS_LSM6DSL_I2C)        += lsm6dsl_i2c.o
obj-$(CONFIG_SENSORS_LSM6DSL_SPI)        += lsm6dsl_spi.o
# magnetic_sensor
obj-$(CONFIG_SENSORS_AK09916C)      += ak09916c.o
obj-$(CONFIG_SENSORS_YAS539)		+= yas_mag_drv-yas539.o yas_mag_kernel.o
# optical_sensor
obj-$(CONFIG_SENSORS_CM36657)       += cm36657.o
obj-$(CONFIG_SENSORS_CM36686)       += cm36686.o
obj-$(CONFIG_SENSORS_TMD3725)       += tmd3725.o
# grip_sensor
obj-$(CONFIG_SENSORS_SX9320)        +=sx9320.o