io = uart
APP = test
APP_SRCS += test.c  lib/cpx/src/com.c lib/cpx/src/cpx.c
APP_INC  += . $(GAP_LIB_PATH)/include lib/cpx/inc

APP_CFLAGS += -O3 -g
APP_CFLAGS += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1

PMSIS_OS ?= freertos

include $(RULES_DIR)/pmsis_rules.mk
