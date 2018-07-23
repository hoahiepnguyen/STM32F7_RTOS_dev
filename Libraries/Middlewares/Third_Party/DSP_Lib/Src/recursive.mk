##############################################################################
#
#   Objects Files
#
##############################################################################

OBJECTS+= ${COMPILER}/arm_rfft_fast_init_f32.o \
		  ${COMPILER}/arm_rfft_fast_f32.o \
          ${COMPILER}/arm_cfft_f32.o \
          ${COMPILER}/arm_cfft_radix8_f32.o \
          ${COMPILER}/arm_bitreversal2.o \
          ${COMPILER}/arm_common_tables.o \
##############################################################################
#
#   The flags passed to the C compiler.
#
##############################################################################
CFLAGS_DEF+=
