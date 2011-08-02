
# setup LCD size

LRESSIZE-$(CONFIG_IG_LCDRES_320x240) = 320x240
LRESSIZE-$(CONFIG_IG_LCDRES_640x480) = 640x480
LRESSIZE-$(CONFIG_IG_LCDRES_800x480) = 800x480
LRESSIZE-$(CONFIG_IG_LCDRES_800x600) = 800x600
LRESSIZE-$(CONFIG_IG_LCDRES_1024x600) = 1024x600
LRESSIZE-$(CONFIG_IG_LCDRES_1024x768) = 1024x768

# Makefile for the Linux logos
obj-y = converted_logo.o

# how to generate logo's

cvert := scripts/convert
to565 := scripts/8882565

# Create commands
cmd_logo = $(cvert) \
		$(srctree)/$(src)/$(CONFIG_IG_LOGO_PATH) -resize '$(LRESSIZE-y)>' \
		-background black -gravity center -extent $(LRESSIZE-y) \
		rgb:$(src)/logo_data

cmd_l2hex = $(to565) \
		$(src)/logo_data $(src)/converted_logo.c

$(obj)/converted_logo.c: $(src)/$(subst ",,$(CONFIG_IG_LOGO_PATH)) $(cvert) $(to565) FORCE
	$(call if_changed,logo)
	$(call if_changed,l2hex)

# Files generated that shall be removed upon make clean
clean-files := *.o *.c logo_data

