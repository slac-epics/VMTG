TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

PROD_IOC = VMTG

DBD += VMTG.dbd
# VMTG.dbd will be made up from these files:
VMTG_DBD += base.dbd
#include definitions for any other support applications needed
#VMTG_DBD += xxx.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
VMTG_SRCS += VMTG_registerRecordDeviceDriver.cpp
VMTG_SRCS_DEFAULT += VMTGMain.cpp
VMTG_SRCS_vxWorks += -nil-

#The following adds support from base/src/vxWorks
VMTG_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

#add a definition for each support application used by this application
#VMTG_LIBS += xxx

#NOTES:
#      1)It is not possible to build sncExample both as a component of VMTG
#        and standalone. You must choose only one.
#      2)To build sncExample SNCSEQ must be defined in <top>/configure/RELEASE

#The following builds sncExample as a component of VMTG
#VMTG_SRCS += sncExample.stt
#VMTG_LIBS += seq pv

VMTG_LIBS += $(EPICS_BASE_IOC_LIBS)

#The following builds sncExample as a standalone application
#PROD_HOST += sncExample
#sncExample_SNCFLAGS += +m
#sncExample_SRCS += sncExample.stt
#sncExample_LIBS += seq pv
#sncExample_LIBS += $(EPICS_BASE_HOST_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

