cmd_arch/arm/mm/cache-v6.o := /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/arm-eabi-gcc -Wp,-MD,arch/arm/mm/.cache-v6.o.d  -nostdinc -isystem /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/../lib/gcc/arm-eabi/4.4.0/include -Iinclude  -I/home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-imapx200/include -Iarch/arm/plat-imap/include -D__ASSEMBLY__ -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables  -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -include asm/unified.h -msoft-float       -c -o arch/arm/mm/cache-v6.o arch/arm/mm/cache-v6.S

deps_arch/arm/mm/cache-v6.o := \
  arch/arm/mm/cache-v6.S \
    $(wildcard include/config/arm/errata/411920.h) \
    $(wildcard include/config/cache/flush/range/limit.h) \
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
    $(wildcard include/config/smp.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/cpu/endian/be8.h) \
    $(wildcard include/config/arm/thumb.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/hwcap.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/unwind.h \
    $(wildcard include/config/arm/unwind.h) \
  arch/arm/mm/proc-macros.S \
    $(wildcard include/config/mmu.h) \
    $(wildcard include/config/cpu/dcache/writethrough.h) \
  include/asm/asm-offsets.h \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/thread_info.h \
    $(wildcard include/config/arm/thumbee.h) \
  /home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include/asm/fpstate.h \
    $(wildcard include/config/vfpv3.h) \
    $(wildcard include/config/iwmmxt.h) \

arch/arm/mm/cache-v6.o: $(deps_arch/arm/mm/cache-v6.o)

$(deps_arch/arm/mm/cache-v6.o):
