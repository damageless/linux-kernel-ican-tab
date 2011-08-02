# sololz
# super mountain callage

obj-$(CONFIG_ACC_RPI1031)	+= rpi1031.o
obj-$(CONFIG_ACC_FS7600)	+= freescale7600.o

EXTRA_CFLAGS += -DLINUX
EXTRA_CFLAGS += -DDIVX_ENABLE

ifeq ($(CONFIG_IMAP_DECODE_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

