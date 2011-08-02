cmd_drivers/video/hdmi/si9022/SIIVideoModeTable.o := /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/arm-eabi-gcc -Wp,-MD,drivers/video/hdmi/si9022/.SIIVideoModeTable.o.d  -nostdinc -isystem /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/../lib/gcc/arm-eabi/4.4.0/include -Iinclude  -I/home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-imapx200/include -Iarch/arm/plat-imap/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fomit-frame-pointer -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fno-dwarf2-cfi-asm -fconserve-stack   -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(SIIVideoModeTable)"  -D"KBUILD_MODNAME=KBUILD_STR(SIIVideoModeTable)"  -c -o drivers/video/hdmi/si9022/SIIVideoModeTable.o drivers/video/hdmi/si9022/SIIVideoModeTable.c

deps_drivers/video/hdmi/si9022/SIIVideoModeTable.o := \
  drivers/video/hdmi/si9022/SIIVideoModeTable.c \
  drivers/video/hdmi/si9022/SIIdefs.h \
  drivers/video/hdmi/si9022/SIITypeDefs.h \
  drivers/video/hdmi/si9022/SIIConstants.h \
    $(wildcard include/config/data/len.h) \
  drivers/video/hdmi/si9022/SIIVideoModeTable.h \

drivers/video/hdmi/si9022/SIIVideoModeTable.o: $(deps_drivers/video/hdmi/si9022/SIIVideoModeTable.o)

$(deps_drivers/video/hdmi/si9022/SIIVideoModeTable.o):
