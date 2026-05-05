#
# Makefile sample for the Cypress USB Type-C and PD Controller
#
# Author: Dudley Du <dudl@cypress.com>
#

obj-$(CONFIG_USBC_CYPRESS_CCG)		+= cyccg.o
cyccg-objs := cyccg_core.o cyccg_hpi.o cyccg_update.o cyccg_sysfs.o	\
	      customer_pd.o
cyccg-$(CONFIG_USBC_CYPRESS_CCG_I2C)	+= cyccg_i2c.o
cyccg-$(CONFIG_USBC_CYPRESS_CCG_SPI)	+= cyccg_spi.o