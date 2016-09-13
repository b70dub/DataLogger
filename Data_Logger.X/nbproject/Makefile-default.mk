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
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Data_Logger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Data_Logger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=main.c mmcPIC32.c ff.c MyToolbox.c Delay_32.c GLOBAL_VARS.c I2C_HardwareDrvr.c MMA8452_I2C.c SystemTimer.c DataLoggingDefs.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main.o ${OBJECTDIR}/mmcPIC32.o ${OBJECTDIR}/ff.o ${OBJECTDIR}/MyToolbox.o ${OBJECTDIR}/Delay_32.o ${OBJECTDIR}/GLOBAL_VARS.o ${OBJECTDIR}/I2C_HardwareDrvr.o ${OBJECTDIR}/MMA8452_I2C.o ${OBJECTDIR}/SystemTimer.o ${OBJECTDIR}/DataLoggingDefs.o
POSSIBLE_DEPFILES=${OBJECTDIR}/main.o.d ${OBJECTDIR}/mmcPIC32.o.d ${OBJECTDIR}/ff.o.d ${OBJECTDIR}/MyToolbox.o.d ${OBJECTDIR}/Delay_32.o.d ${OBJECTDIR}/GLOBAL_VARS.o.d ${OBJECTDIR}/I2C_HardwareDrvr.o.d ${OBJECTDIR}/MMA8452_I2C.o.d ${OBJECTDIR}/SystemTimer.o.d ${OBJECTDIR}/DataLoggingDefs.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main.o ${OBJECTDIR}/mmcPIC32.o ${OBJECTDIR}/ff.o ${OBJECTDIR}/MyToolbox.o ${OBJECTDIR}/Delay_32.o ${OBJECTDIR}/GLOBAL_VARS.o ${OBJECTDIR}/I2C_HardwareDrvr.o ${OBJECTDIR}/MMA8452_I2C.o ${OBJECTDIR}/SystemTimer.o ${OBJECTDIR}/DataLoggingDefs.o

# Source Files
SOURCEFILES=main.c mmcPIC32.c ff.c MyToolbox.c Delay_32.c GLOBAL_VARS.c I2C_HardwareDrvr.c MMA8452_I2C.c SystemTimer.c DataLoggingDefs.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Data_Logger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX440F256H
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mmcPIC32.o: mmcPIC32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/mmcPIC32.o.d 
	@${RM} ${OBJECTDIR}/mmcPIC32.o 
	@${FIXDEPS} "${OBJECTDIR}/mmcPIC32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/mmcPIC32.o.d" -o ${OBJECTDIR}/mmcPIC32.o mmcPIC32.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/ff.o: ff.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ff.o.d 
	@${RM} ${OBJECTDIR}/ff.o 
	@${FIXDEPS} "${OBJECTDIR}/ff.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/ff.o.d" -o ${OBJECTDIR}/ff.o ff.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/MyToolbox.o: MyToolbox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MyToolbox.o.d 
	@${RM} ${OBJECTDIR}/MyToolbox.o 
	@${FIXDEPS} "${OBJECTDIR}/MyToolbox.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/MyToolbox.o.d" -o ${OBJECTDIR}/MyToolbox.o MyToolbox.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Delay_32.o: Delay_32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Delay_32.o.d 
	@${RM} ${OBJECTDIR}/Delay_32.o 
	@${FIXDEPS} "${OBJECTDIR}/Delay_32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/Delay_32.o.d" -o ${OBJECTDIR}/Delay_32.o Delay_32.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/GLOBAL_VARS.o: GLOBAL_VARS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/GLOBAL_VARS.o.d 
	@${RM} ${OBJECTDIR}/GLOBAL_VARS.o 
	@${FIXDEPS} "${OBJECTDIR}/GLOBAL_VARS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/GLOBAL_VARS.o.d" -o ${OBJECTDIR}/GLOBAL_VARS.o GLOBAL_VARS.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/I2C_HardwareDrvr.o: I2C_HardwareDrvr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2C_HardwareDrvr.o.d 
	@${RM} ${OBJECTDIR}/I2C_HardwareDrvr.o 
	@${FIXDEPS} "${OBJECTDIR}/I2C_HardwareDrvr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/I2C_HardwareDrvr.o.d" -o ${OBJECTDIR}/I2C_HardwareDrvr.o I2C_HardwareDrvr.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/MMA8452_I2C.o: MMA8452_I2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MMA8452_I2C.o.d 
	@${RM} ${OBJECTDIR}/MMA8452_I2C.o 
	@${FIXDEPS} "${OBJECTDIR}/MMA8452_I2C.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/MMA8452_I2C.o.d" -o ${OBJECTDIR}/MMA8452_I2C.o MMA8452_I2C.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/SystemTimer.o: SystemTimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SystemTimer.o.d 
	@${RM} ${OBJECTDIR}/SystemTimer.o 
	@${FIXDEPS} "${OBJECTDIR}/SystemTimer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/SystemTimer.o.d" -o ${OBJECTDIR}/SystemTimer.o SystemTimer.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/DataLoggingDefs.o: DataLoggingDefs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/DataLoggingDefs.o.d 
	@${RM} ${OBJECTDIR}/DataLoggingDefs.o 
	@${FIXDEPS} "${OBJECTDIR}/DataLoggingDefs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/DataLoggingDefs.o.d" -o ${OBJECTDIR}/DataLoggingDefs.o DataLoggingDefs.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/mmcPIC32.o: mmcPIC32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/mmcPIC32.o.d 
	@${RM} ${OBJECTDIR}/mmcPIC32.o 
	@${FIXDEPS} "${OBJECTDIR}/mmcPIC32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/mmcPIC32.o.d" -o ${OBJECTDIR}/mmcPIC32.o mmcPIC32.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/ff.o: ff.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ff.o.d 
	@${RM} ${OBJECTDIR}/ff.o 
	@${FIXDEPS} "${OBJECTDIR}/ff.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/ff.o.d" -o ${OBJECTDIR}/ff.o ff.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/MyToolbox.o: MyToolbox.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MyToolbox.o.d 
	@${RM} ${OBJECTDIR}/MyToolbox.o 
	@${FIXDEPS} "${OBJECTDIR}/MyToolbox.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/MyToolbox.o.d" -o ${OBJECTDIR}/MyToolbox.o MyToolbox.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/Delay_32.o: Delay_32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Delay_32.o.d 
	@${RM} ${OBJECTDIR}/Delay_32.o 
	@${FIXDEPS} "${OBJECTDIR}/Delay_32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/Delay_32.o.d" -o ${OBJECTDIR}/Delay_32.o Delay_32.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/GLOBAL_VARS.o: GLOBAL_VARS.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/GLOBAL_VARS.o.d 
	@${RM} ${OBJECTDIR}/GLOBAL_VARS.o 
	@${FIXDEPS} "${OBJECTDIR}/GLOBAL_VARS.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/GLOBAL_VARS.o.d" -o ${OBJECTDIR}/GLOBAL_VARS.o GLOBAL_VARS.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/I2C_HardwareDrvr.o: I2C_HardwareDrvr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2C_HardwareDrvr.o.d 
	@${RM} ${OBJECTDIR}/I2C_HardwareDrvr.o 
	@${FIXDEPS} "${OBJECTDIR}/I2C_HardwareDrvr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/I2C_HardwareDrvr.o.d" -o ${OBJECTDIR}/I2C_HardwareDrvr.o I2C_HardwareDrvr.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/MMA8452_I2C.o: MMA8452_I2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MMA8452_I2C.o.d 
	@${RM} ${OBJECTDIR}/MMA8452_I2C.o 
	@${FIXDEPS} "${OBJECTDIR}/MMA8452_I2C.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/MMA8452_I2C.o.d" -o ${OBJECTDIR}/MMA8452_I2C.o MMA8452_I2C.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/SystemTimer.o: SystemTimer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SystemTimer.o.d 
	@${RM} ${OBJECTDIR}/SystemTimer.o 
	@${FIXDEPS} "${OBJECTDIR}/SystemTimer.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/SystemTimer.o.d" -o ${OBJECTDIR}/SystemTimer.o SystemTimer.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/DataLoggingDefs.o: DataLoggingDefs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/DataLoggingDefs.o.d 
	@${RM} ${OBJECTDIR}/DataLoggingDefs.o 
	@${FIXDEPS} "${OBJECTDIR}/DataLoggingDefs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -I"pic32mx/include" -I"pic32mx/include/peripheral" -MMD -MF "${OBJECTDIR}/DataLoggingDefs.o.d" -o ${OBJECTDIR}/DataLoggingDefs.o DataLoggingDefs.c    -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Data_Logger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Data_Logger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)    -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Data_Logger.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Data_Logger.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--no-code-in-dinit,--no-dinit-in-serial-mem,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Data_Logger.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
