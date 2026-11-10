TOP = ../..
include $(TOP)/configure/CONFIG
DBDINC += ditherRecord
LIBRARY_IOC += motorDither
DBD += motorDither.dbd
motorDither_SRCS += ditherRecord.c
motorDither_LIBS += $(EPICS_BASE_IOC_LIBS)
include $(TOP)/configure/RULES
