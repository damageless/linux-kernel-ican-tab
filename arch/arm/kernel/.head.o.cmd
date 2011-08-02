cmd_arch/arm/kernel/head.o := /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/arm-eabi-gcc -Wp,-MD,arch/arm/kernel/.head.o.d  -nostdinc -isystem /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/../lib/gcc/arm-eabi/4.4.0/include -Iinclude  -I/home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-imapx200/include -Iarch/arm/plat-imap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables  -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -include asm/unified.h -msoft-float     -DTEXT_OFFSET=0x00008000  -c -o arch/arm/kernel/head.o arch/arm/kernel/head.S

deps_arch/arm/kernel/head.o := \
  arch/arm/kernel/head.S \
    $(wildcard include/config/xip/kernel.h) \
    $(wildcard include/config/xip/phys/addr.h) \
    $(wildcard include/config/smp.h) \
    $(wildcard include/config/alignment/trap.h) \
    $(wildcard include/config/cpu/dcache/disable.h) \
    $(wildcard include/config/cpu/bpredict/disable.h) \
    $(wildcard include/config/cpu/icache/disable.h) \
    $(wildcard include/config/debug/ll.h) \
    $(wildcard include/config/arch/netwinder.h) \
    $(wildcard include/config/arch/cats.h) \
    $(wildcard include/config/arch/rpc.h) \
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
  include/linux/init.h \
    $(wildcard include/config/modules.h) \
    $(wildcard include/config/hotplug.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/assembler.h \
    $(wildcard include/config/cpu/feroceon.h) \
    $(wildcard include/config/trace/irqflags.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/hwcap.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/domain.h \
    $(wildcard include/config/io/36.h) \
    $(wildcard include/config/mmu.h) \
  include/asm/asm-offsets.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/memory.h \
    $(wildcard include/config/page/offset.h) \
    $(wildcard include/config/highmem.h) \
    $(wildcard include/config/dram/size.h) \
    $(wildcard include/config/dram/base.h) \
    $(wildcard include/config/zone/dma.h) \
    $(wildcard include/config/discontigmem.h) \
  include/linux/const.h \
  arch/arm/mach-imapx200/include/mach/memory.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/sizes.h \
  include/asm-generic/memory_model.h \
    $(wildcard include/config/flatmem.h) \
    $(wildcard include/config/sparsemem/vmemmap.h) \
    $(wildcard include/config/sparsemem.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/thread_info.h \
    $(wildcard include/config/arm/thumbee.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/fpstate.h \
    $(wildcard include/config/vfpv3.h) \
    $(wildcard include/config/iwmmxt.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/system.h \
    $(wildcard include/config/cpu/xsc3.h) \
    $(wildcard include/config/cpu/fa526.h) \
    $(wildcard include/config/cpu/sa1100.h) \
    $(wildcard include/config/cpu/sa110.h) \
    $(wildcard include/config/cpu/32v6k.h) \
  arch/arm/kernel/head-common.S \

arch/arm/kernel/head.o: $(deps_arch/arm/kernel/head.o)

$(deps_arch/arm/kernel/head.o):
