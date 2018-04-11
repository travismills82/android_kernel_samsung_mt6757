#
# Makefile for the stui Module
#
#EXTRA_CFLAGS += -Idrivers/misc/tui
ccflags-y += -Idrivers/misc/tui
ccflags-y += -I$(srctree)/drivers/staging/android/ion
ccflags-y += -I$(srctree)/drivers/staging/android/ion/mtk
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat

obj-$(CONFIG_SAMSUNG_TUI)	+= main.o
obj-$(CONFIG_SAMSUNG_TUI)	+= stui_core.o
obj-$(CONFIG_SAMSUNG_TUI)	+= stui_inf.o
obj-$(CONFIG_SAMSUNG_TUI)	+= ./platforms/mt6757/stui_hal_display.o
obj-$(CONFIG_SAMSUNG_TUI)	+= ./platforms/mt6757/stui_hal_touch.o

ccflags-y += -DTUI_ENABLE_DISPLAY
ccflags-y += -DTUI_ENABLE_TOUCH
ccflags-y += -DTUI_ENABLE_I2C
ccflags-y += -DTUI_ENABLE_MEMORY_SSVP

ccflags-$(CONFIG_SAMSUNG_TUI_TEST) += -DSAMSUNG_TUI_TEST
obj-$(CONFIG_SAMSUNG_TUI_TEST)     += test_driver/