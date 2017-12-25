##############################################################################
#
#   Objects Files
#
##############################################################################

OBJECTS+=${COMPILER}/stm32f7xx_hal.o \
         ${COMPILER}/stm32f7xx_hal_gpio.o \
         ${COMPILER}/stm32f7xx_hal_rcc.o \
         ${COMPILER}/stm32f7xx_hal_cortex.o \
         ${COMPILER}/stm32f7xx_hal_uart.o \
         ${COMPILER}/stm32f7xx_hal_pwr_ex.o \
		 ${COMPILER}/stm32f7xx_hal_i2c.o \
		 ${COMPILER}/stm32f7xx_hal_dma.o \
		 ${COMPILER}/stm32f7xx_hal_rcc_ex.o \
		 ${COMPILER}/stm32f7xx_hal_i2c_ex.o \

##############################################################################
#
#   The flags passed to the C compiler.
#
##############################################################################
CFLAGS_DEF+=
