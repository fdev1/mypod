#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/mypod.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/mypod.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../audio.c ../audio_asm.s ../main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/audio.o ${OBJECTDIR}/_ext/1472/audio_asm.o ${OBJECTDIR}/_ext/1472/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/audio.o.d ${OBJECTDIR}/_ext/1472/audio_asm.o.d ${OBJECTDIR}/_ext/1472/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/audio.o ${OBJECTDIR}/_ext/1472/audio_asm.o ${OBJECTDIR}/_ext/1472/main.o

# Source Files
SOURCEFILES=../audio.c ../audio_asm.s ../main.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/mypod.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP512GP502
MP_LINKER_FILE_OPTION=,--script=p33EP512GP502.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/audio.o: ../audio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/audio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/audio.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../audio.c  -o ${OBJECTDIR}/_ext/1472/audio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/audio.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=coff -mlarge-code -mlarge-data -mconst-in-code -O0 -msmart-io=1 -Wall -msfr-warn=off -I"../../fat32lib" -I"../../lglib" -I"../../fat32lib/dspic_hal" -I"../../mp3lib"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/audio.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"      -g -D__DEBUG   -mno-eds-warn  -omf=coff -mlarge-code -mlarge-data -mconst-in-code -O0 -msmart-io=1 -Wall -msfr-warn=off -I"../../fat32lib" -I"../../lglib" -I"../../fat32lib/dspic_hal" -I"../../mp3lib"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1472/audio.o: ../audio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/audio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/audio.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../audio.c  -o ${OBJECTDIR}/_ext/1472/audio.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/audio.o.d"      -mno-eds-warn  -g -omf=coff -mlarge-code -mlarge-data -mconst-in-code -O0 -msmart-io=1 -Wall -msfr-warn=off -I"../../fat32lib" -I"../../lglib" -I"../../fat32lib/dspic_hal" -I"../../mp3lib"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/audio.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../main.c  -o ${OBJECTDIR}/_ext/1472/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d"      -mno-eds-warn  -g -omf=coff -mlarge-code -mlarge-data -mconst-in-code -O0 -msmart-io=1 -Wall -msfr-warn=off -I"../../fat32lib" -I"../../lglib" -I"../../fat32lib/dspic_hal" -I"../../mp3lib"
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/audio_asm.o: ../audio_asm.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/audio_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/audio_asm.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../audio_asm.s  -o ${OBJECTDIR}/_ext/1472/audio_asm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG   -omf=coff -Wa,-MD,"${OBJECTDIR}/_ext/1472/audio_asm.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/audio_asm.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/_ext/1472/audio_asm.o: ../audio_asm.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/audio_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/audio_asm.o 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../audio_asm.s  -o ${OBJECTDIR}/_ext/1472/audio_asm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=coff -Wa,-MD,"${OBJECTDIR}/_ext/1472/audio_asm.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/audio_asm.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/mypod.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../fat32lib/fat32lib/fat32lib_p33e.a ../../fat32lib/sdlib/sdlib_p33e.a ../../fat32lib/smlib/smlib_p33e.a ../../mp3lib/mp3lib_p33e.a ../../lglib/ili9341/ili9341_p33e.a ../../lglib/lglib/lglib_p33e.a ../../fat32lib/dspic_hal/dspic_hal_p33e.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/mypod.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ../../fat32lib/fat32lib/fat32lib_p33e.a ../../fat32lib/sdlib/sdlib_p33e.a ../../fat32lib/smlib/smlib_p33e.a ../../mp3lib/mp3lib_p33e.a ../../lglib/ili9341/ili9341_p33e.a ../../lglib/lglib/lglib_p33e.a ../../fat32lib/dspic_hal/dspic_hal_p33e.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG   -omf=coff -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,,$(MP_LINKER_FILE_OPTION),--heap=2048,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/mypod.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../fat32lib/fat32lib/fat32lib_p33e.a ../../fat32lib/sdlib/sdlib_p33e.a ../../fat32lib/smlib/smlib_p33e.a ../../mp3lib/mp3lib_p33e.a ../../lglib/ili9341/ili9341_p33e.a ../../lglib/lglib/lglib_p33e.a ../../fat32lib/dspic_hal/dspic_hal_p33e.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/mypod.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ../../fat32lib/fat32lib/fat32lib_p33e.a ../../fat32lib/sdlib/sdlib_p33e.a ../../fat32lib/smlib/smlib_p33e.a ../../mp3lib/mp3lib_p33e.a ../../lglib/ili9341/ili9341_p33e.a ../../lglib/lglib/lglib_p33e.a ../../fat32lib/dspic_hal/dspic_hal_p33e.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=coff -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--heap=2048,--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}/xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/mypod.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=coff  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
