cmd_arch/arm/plat-imap/sleep.o := /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/arm-eabi-gcc -Wp,-MD,arch/arm/plat-imap/.sleep.o.d  -nostdinc -isystem /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/../lib/gcc/arm-eabi/4.4.0/include -Iinclude  -I/home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-imapx200/include -Iarch/arm/plat-imap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables  -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -include asm/unified.h -msoft-float       -c -o arch/arm/plat-imap/sleep.o arch/arm/plat-imap/sleep.S

deps_arch/arm/plat-imap/sleep.o := \
  arch/arm/plat-imap/sleep.S \
    $(wildcard include/config/imap/lowlevel/uart/port.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/unified.h \
    $(wildcard include/config/arm/asm/unified.h) \
    $(wildcard include/config/thumb2/kernel.h) \
  include/linux/linkage.h \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/linkage.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/assembler.h \
    $(wildcard include/config/cpu/feroceon.h) \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/smp.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/hwcap.h \
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

arch/arm/plat-imap/sleep.o: $(deps_arch/arm/plat-imap/sleep.o)

$(deps_arch/arm/plat-imap/sleep.o):
