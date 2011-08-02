cmd_arch/arm/boot/compressed/misc.o := /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/arm-eabi-gcc -Wp,-MD,arch/arm/boot/compressed/.misc.o.d  -nostdinc -isystem /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/../lib/gcc/arm-eabi/4.4.0/include -Iinclude  -I/home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-imapx200/include -Iarch/arm/plat-imap/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fomit-frame-pointer -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fno-dwarf2-cfi-asm -fconserve-stack -fpic -fno-builtin -Dstatic=   -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(misc)"  -D"KBUILD_MODNAME=KBUILD_STR(misc)"  -c -o arch/arm/boot/compressed/misc.o arch/arm/boot/compressed/misc.c

deps_arch/arm/boot/compressed/misc.o := \
  arch/arm/boot/compressed/misc.c \
    $(wildcard include/config/debug/icedcc.h) \
    $(wildcard include/config/cpu/v6.h) \
    $(wildcard include/config/cpu/v7.h) \
    $(wildcard include/config/cpu/xscale.h) \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/compiler-gcc.h \
    $(wildcard include/config/arch/supports/optimized/inlining.h) \
    $(wildcard include/config/optimize/inlining.h) \
  include/linux/compiler-gcc4.h \
  include/linux/types.h \
    $(wildcard include/config/uid16.h) \
    $(wildcard include/config/lbdaf.h) \
    $(wildcard include/config/phys/addr/t/64bit.h) \
    $(wildcard include/config/64bit.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/types.h \
  include/asm-generic/int-ll64.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/bitsperlong.h \
  include/asm-generic/bitsperlong.h \
  include/linux/posix_types.h \
  include/linux/stddef.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/posix_types.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/string.h \
  arch/arm/mach-imapx200/include/mach/uncompress.h \
  arch/arm/mach-imapx200/include/mach/imap_addr.h \
  arch/arm/plat-imap/include/plat/imapx.h \
  arch/arm/mach-imapx200/include/mach/imapx_sysmgr.h \
  arch/arm/mach-imapx200/include/mach/imapx_dma.h \
  arch/arm/mach-imapx200/include/mach/imapx_nand.h \
  arch/arm/mach-imapx200/include/mach/imapx_sdio.h \
  arch/arm/mach-imapx200/include/mach/imapx_cf.h \
  arch/arm/mach-imapx200/include/mach/imapx_usbhost.h \
  arch/arm/mach-imapx200/include/mach/imapx_usbotg.h \
  arch/arm/mach-imapx200/include/mach/imapx_intr.h \
  arch/arm/mach-imapx200/include/mach/imapx_cam.h \
  arch/arm/mach-imapx200/include/mach/imapx_lcd.h \
  arch/arm/mach-imapx200/include/mach/imapx_mac.h \
  arch/arm/mach-imapx200/include/mach/imapx_graphic.h \
  arch/arm/mach-imapx200/include/mach/imapx_venc.h \
  arch/arm/mach-imapx200/include/mach/imapx_vdec.h \
  arch/arm/mach-imapx200/include/mach/imapx_timer.h \
  arch/arm/mach-imapx200/include/mach/imapx_pwm.h \
  arch/arm/mach-imapx200/include/mach/imapx_wdog.h \
  arch/arm/mach-imapx200/include/mach/imapx_iic.h \
  arch/arm/mach-imapx200/include/mach/imapx_rtc.h \
  arch/arm/mach-imapx200/include/mach/imapx_iis.h \
  arch/arm/mach-imapx200/include/mach/imapx_ac97.h \
  arch/arm/mach-imapx200/include/mach/imapx_spi.h \
  arch/arm/mach-imapx200/include/mach/imapx_gpio.h \
  arch/arm/mach-imapx200/include/mach/imapx_uart.h \
  arch/arm/mach-imapx200/include/mach/imapx_keybd.h \
  arch/arm/mach-imapx200/include/mach/imapx_ps2.h \
  arch/arm/mach-imapx200/include/mach/imapx_idsp.h \
  arch/arm/mach-imapx200/include/mach/imapx_base_reg.h \
  arch/arm/plat-imap/include/plat/uncompress.h \
    $(wildcard include/config/s3c/lowlevel/uart/port.h) \
    $(wildcard include/config/imap/lowlevel/uart/port.h) \
    $(wildcard include/config/imap/boot/watchdog.h) \
    $(wildcard include/config/s3c/boot/error/reset.h) \
  arch/arm/plat-imap/include/plat/fpga_test.h \
  arch/arm/plat-imap/include/plat/regs-serial.h \
  arch/arm/plat-imap/include/plat/regs-watchdog.h \
  arch/arm/boot/compressed/../../../../lib/inflate.c \

arch/arm/boot/compressed/misc.o: $(deps_arch/arm/boot/compressed/misc.o)

$(deps_arch/arm/boot/compressed/misc.o):
