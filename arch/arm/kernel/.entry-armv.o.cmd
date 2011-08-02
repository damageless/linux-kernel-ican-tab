cmd_arch/arm/kernel/entry-armv.o := /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/arm-eabi-gcc -Wp,-MD,arch/arm/kernel/.entry-armv.o.d  -nostdinc -isystem /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/../lib/gcc/arm-eabi/4.4.0/include -Iinclude  -I/home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-imapx200/include -Iarch/arm/plat-imap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables  -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -include asm/unified.h -msoft-float       -c -o arch/arm/kernel/entry-armv.o arch/arm/kernel/entry-armv.S

deps_arch/arm/kernel/entry-armv.o := \
  arch/arm/kernel/entry-armv.S \
    $(wildcard include/config/smp.h) \
    $(wildcard include/config/local/timers.h) \
    $(wildcard include/config/kprobes.h) \
    $(wildcard include/config/aeabi.h) \
    $(wildcard include/config/thumb2/kernel.h) \
    $(wildcard include/config/preempt.h) \
    $(wildcard include/config/trace/irqflags.h) \
    $(wildcard include/config/needs/syscall/for/cmpxchg.h) \
    $(wildcard include/config/mmu.h) \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/neon.h) \
    $(wildcard include/config/cpu/arm610.h) \
    $(wildcard include/config/cpu/arm710.h) \
    $(wildcard include/config/iwmmxt.h) \
    $(wildcard include/config/crunch.h) \
    $(wildcard include/config/vfp.h) \
    $(wildcard include/config/has/tls/reg.h) \
    $(wildcard include/config/tls/reg/emul.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/unified.h \
    $(wildcard include/config/arm/asm/unified.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/memory.h \
    $(wildcard include/config/page/offset.h) \
    $(wildcard include/config/highmem.h) \
    $(wildcard include/config/dram/size.h) \
    $(wildcard include/config/dram/base.h) \
    $(wildcard include/config/zone/dma.h) \
    $(wildcard include/config/discontigmem.h) \
  include/linux/compiler.h \
    $(wildcard include/config/trace/branch/profiling.h) \
    $(wildcard include/config/profile/all/branches.h) \
    $(wildcard include/config/enable/must/check.h) \
    $(wildcard include/config/enable/warn/deprecated.h) \
  include/linux/const.h \
  arch/arm/mach-imapx200/include/mach/memory.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/sizes.h \
  include/asm-generic/memory_model.h \
    $(wildcard include/config/flatmem.h) \
    $(wildcard include/config/sparsemem/vmemmap.h) \
    $(wildcard include/config/sparsemem.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/glue.h \
    $(wildcard include/config/cpu/abrt/lv4t.h) \
    $(wildcard include/config/cpu/abrt/ev4.h) \
    $(wildcard include/config/cpu/abrt/ev4t.h) \
    $(wildcard include/config/cpu/abrt/ev5tj.h) \
    $(wildcard include/config/cpu/abrt/ev5t.h) \
    $(wildcard include/config/cpu/abrt/ev6.h) \
    $(wildcard include/config/cpu/abrt/ev7.h) \
    $(wildcard include/config/cpu/pabrt/legacy.h) \
    $(wildcard include/config/cpu/pabrt/v6.h) \
    $(wildcard include/config/cpu/pabrt/v7.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/vfpmacros.h \
    $(wildcard include/config/vfpv3.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/vfp.h \
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
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/thread_notify.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/unwind.h \
    $(wildcard include/config/arm/unwind.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/unistd.h \
    $(wildcard include/config/oabi/compat.h) \
  arch/arm/kernel/entry-header.S \
    $(wildcard include/config/frame/pointer.h) \
    $(wildcard include/config/alignment/trap.h) \
    $(wildcard include/config/cpu/32v6k.h) \
    $(wildcard include/config/cpu/v6.h) \
  include/linux/init.h \
    $(wildcard include/config/modules.h) \
    $(wildcard include/config/hotplug.h) \
  include/linux/linkage.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/linkage.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/assembler.h \
    $(wildcard include/config/cpu/feroceon.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/ptrace.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/hwcap.h \
  include/asm/asm-offsets.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/errno.h \
  include/asm-generic/errno.h \
  include/asm-generic/errno-base.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/thread_info.h \
    $(wildcard include/config/arm/thumbee.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/fpstate.h \

arch/arm/kernel/entry-armv.o: $(deps_arch/arm/kernel/entry-armv.o)

$(deps_arch/arm/kernel/entry-armv.o):
