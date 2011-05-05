TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

LIBRARY_IOC_RTEMS = VMTGSup
PROD_IOC_RTEMS = VMTG

DBD += VMTG.dbd
DBD += VMTGSup.dbd
# VMTG.dbd will be made up from these files:
VMTG_DBD += base.dbd
VMTG_DBD += VMTGSup.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
VMTG_SRCS += VMTG_registerRecordDeviceDriver.cpp
VMTG_SRCS_DEFAULT += VMTGMain.cpp
VMTG_SRCS_RTEMS   += -nil-

#add a definition for each support application used by this application
VMTG_LIBS += VMTGSup

#NOTES:
#      1)It is not possible to build sncExample both as a component of VMTG
#        and standalone. You must choose only one.
#      2)To build sncExample SNCSEQ must be defined in <top>/configure/RELEASE

#The following builds sncExample as a component of VMTG
VMTGSup_SRCS   += vme64x.c
VMTGSup_SRCS   += drvVMTG.c
SRC_DIRS       += ../vme64xSup

VMTG_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

