TOP = .
include $(TOP)/configure/CONFIG
DIRS += configure ditherApp
ditherApp_DEPEND_DIRS += configure
include $(TOP)/configure/RULES_TOP
