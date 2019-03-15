#
# Makefile for the contributed drivers
#
CONTRIB_DRIVERS_SRC:=$(TOPDIR)/contrib/driver

INCL  += -I$(CONTRIB_DRIVERS_SRC)/

COBJS-$(CONFIG_TWI0)		+= $(CONTRIB_DRIVERS_SRC)/muxHwInit.o
COBJS-$(CONFIG_TWI0)		+= $(CONTRIB_DRIVERS_SRC)/muxHwFpga.o
COBJS-$(CONFIG_TWI0)		+= $(CONTRIB_DRIVERS_SRC)/muxHwUtils.o

#COBJS-$(CONFIG_TWI0)		+= $(CONTRIB_DRIVERS_SRC)/pwmc/pwmc.o


