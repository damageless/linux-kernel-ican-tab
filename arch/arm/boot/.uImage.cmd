cmd_arch/arm/boot/uImage := /bin/bash /home/adam/Downloads/android-linux-2.6.32.9-union-release/scripts/mkuboot.sh -A arm -O linux -T kernel -C none -a 0x40008000 -e 0x40008000 -n 'Linux-2.6.32.9' -d arch/arm/boot/zImage arch/arm/boot/uImage