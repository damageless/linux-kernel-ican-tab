cmd_drivers/i2c/chips/smb380calib.o := /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/arm-eabi-gcc -Wp,-MD,drivers/i2c/chips/.smb380calib.o.d  -nostdinc -isystem /home/adam/android/android-ndk-r5b/toolchains/arm-eabi-4.4.0/prebuilt/linux-x86/bin/../lib/gcc/arm-eabi/4.4.0/include -Iinclude  -I/home/adam/Downloads/android-linux-2.6.32.9-union-release/arch/arm/include -include include/linux/autoconf.h -D__KERNEL__ -mlittle-endian -Iarch/arm/mach-imapx200/include -Iarch/arm/plat-imap/include -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration -Wno-format-security -fno-delete-null-pointer-checks -Os -marm -mabi=aapcs-linux -mno-thumb-interwork -funwind-tables -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -Wframe-larger-than=1024 -fno-stack-protector -fomit-frame-pointer -Wdeclaration-after-statement -Wno-pointer-sign -fno-strict-overflow -fno-dwarf2-cfi-asm -fconserve-stack   -D"KBUILD_STR(s)=\#s" -D"KBUILD_BASENAME=KBUILD_STR(smb380calib)"  -D"KBUILD_MODNAME=KBUILD_STR(smb380calib)"  -c -o drivers/i2c/chips/smb380calib.o drivers/i2c/chips/smb380calib.c

deps_drivers/i2c/chips/smb380calib.o := \
  drivers/i2c/chips/smb380calib.c \
  drivers/i2c/chips/smb380.h \
  drivers/i2c/chips/smb380calib.h \

drivers/i2c/chips/smb380calib.o: $(deps_drivers/i2c/chips/smb380calib.o)

$(deps_drivers/i2c/chips/smb380calib.o):
