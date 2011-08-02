cmd_arch/arm/kernel/entry-common.o := /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/arm-eabi-gcc -Wp,-MD,arch/arm/kernel/.entry-common.o.d  -nostdinc -isystem /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/../lib/gcc/arm-eabi/4.4.0/include -Iinclude  -I/home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-imapx200/include -Iarch/arm/plat-imap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables  -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -include asm/unified.h -msoft-float       -c -o arch/arm/kernel/entry-common.o arch/arm/kernel/entry-common.S

deps_arch/arm/kernel/entry-common.o := \
  arch/arm/kernel/entry-common.S \
    $(wildcard include/config/function/tracer.h) \
    $(wildcard include/config/dynamic/ftrace.h) \
    $(wildcard include/config/cpu/arm710.h) \
    $(wildcard include/config/oabi/compat.h) \
    $(wildcard include/config/arm/thumb.h) \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/aeabi.h) \
    $(wildcard include/config/alignment/trap.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/unified.h \
    $(wildcard include/config/arm/asm/unified.h) \
    $(wildcard include/config/thumb2/kernel.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/unistd.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/ftrace.h \
    $(wildcard include/config/frame/pointer.h) \
    $(wildcard include/config/arm/unwind.h) \
  arch/arm/mach-imapx200/include/mach/entry-macro.S \
    $(wildcard include/config/cpu/imapx200.h) \
  arch/arm/mach-imapx200/include/mach/hardware.h \
    $(wildcard include/config/no/multiword/io.h) \
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
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/irq.h \
  arch/arm/mach-imapx200/include/mach/irqs.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/unwind.h \
  arch/arm/kernel/entry-header.S \
    $(wildcard include/config/cpu/32v6k.h) \
    $(wildcard include/config/cpu/v6.h) \
  include/linux/init.h \
    $(wildcard include/config/modules.h) \
    $(wildcard include/config/hotplug.h) \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/linkage.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/linkage.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/assembler.h \
    $(wildcard include/config/cpu/feroceon.h) \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/smp.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/ptrace.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/hwcap.h \
  include/asm/asm-offsets.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/errno.h \
  include/asm-generic/errno.h \
  include/asm-generic/errno-base.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/thread_info.h \
    $(wildcard include/config/arm/thumbee.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/fpstate.h \
    $(wildcard include/config/vfpv3.h) \
    $(wildcard include/config/iwmmxt.h) \
  arch/arm/kernel/calls.S \

arch/arm/kernel/entry-common.o: $(deps_arch/arm/kernel/entry-common.o)

$(deps_arch/arm/kernel/entry-common.o):
