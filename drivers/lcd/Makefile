#
# Seva Engenharia Eletronica S.A
#

include $(TOPDIR)/config.mk

LIB	:= $(obj)liblcd.o

COBJS-$(CONFIG_LCD_TCM) += rasterDisplay.o interrupt.o lcd.o raster.o

COBJS	:= $(COBJS-y)
SRCS	:= $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS))

all:	$(LIB)

$(LIB):	$(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
