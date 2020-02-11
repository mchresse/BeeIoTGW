#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/beeiotgw/BIoTApp.o \
	${OBJECTDIR}/beeiotgw/JoinSrv.o \
	${OBJECTDIR}/beeiotgw/NwSrv.o \
	${OBJECTDIR}/beeiotgw/aes.o \
	${OBJECTDIR}/beeiotgw/base64.o \
	${OBJECTDIR}/beeiotgw/beelog.o \
	${OBJECTDIR}/beeiotgw/main.o \
	${OBJECTDIR}/beeiotgw/radio.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-lwiringPi

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/beeiotgw

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/beeiotgw: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/beeiotgw ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/beeiotgw/BIoTApp.o: beeiotgw/BIoTApp.cpp
	${MKDIR} -p ${OBJECTDIR}/beeiotgw
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++98 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/beeiotgw/BIoTApp.o beeiotgw/BIoTApp.cpp

${OBJECTDIR}/beeiotgw/JoinSrv.o: beeiotgw/JoinSrv.cpp
	${MKDIR} -p ${OBJECTDIR}/beeiotgw
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++98 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/beeiotgw/JoinSrv.o beeiotgw/JoinSrv.cpp

${OBJECTDIR}/beeiotgw/NwSrv.o: beeiotgw/NwSrv.cpp
	${MKDIR} -p ${OBJECTDIR}/beeiotgw
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++98 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/beeiotgw/NwSrv.o beeiotgw/NwSrv.cpp

${OBJECTDIR}/beeiotgw/aes.o: beeiotgw/aes.cpp
	${MKDIR} -p ${OBJECTDIR}/beeiotgw
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++98 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/beeiotgw/aes.o beeiotgw/aes.cpp

${OBJECTDIR}/beeiotgw/base64.o: beeiotgw/base64.cpp
	${MKDIR} -p ${OBJECTDIR}/beeiotgw
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++98 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/beeiotgw/base64.o beeiotgw/base64.cpp

${OBJECTDIR}/beeiotgw/beelog.o: beeiotgw/beelog.cpp
	${MKDIR} -p ${OBJECTDIR}/beeiotgw
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++98 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/beeiotgw/beelog.o beeiotgw/beelog.cpp

${OBJECTDIR}/beeiotgw/main.o: beeiotgw/main.cpp
	${MKDIR} -p ${OBJECTDIR}/beeiotgw
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++98 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/beeiotgw/main.o beeiotgw/main.cpp

${OBJECTDIR}/beeiotgw/radio.o: beeiotgw/radio.cpp
	${MKDIR} -p ${OBJECTDIR}/beeiotgw
	${RM} "$@.d"
	$(COMPILE.cc) -g -std=c++98 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/beeiotgw/radio.o beeiotgw/radio.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
