################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../__vm/


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../SparkFunLSM9DS1.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
SparkFunLSM9DS1.o

OBJS_AS_ARGS +=  \
SparkFunLSM9DS1.o

C_DEPS +=  \
SparkFunLSM9DS1.d

C_DEPS_AS_ARGS +=  \
SparkFunLSM9DS1.d

OUTPUT_FILE_PATH +=IMU_SpinModule.elf

OUTPUT_FILE_PATH_AS_ARGS +=IMU_SpinModule.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler



./%.o: .././%.cpp
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 5.3.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-g++.exe$(QUOTE) -mthumb -D__SAM3X8E__ -DARDUINO=10609 -DARDUINO_MAIN -DF_CPU=84000000L -Dprintf=iprintf -D__SAM__ -DARDUINO_SAM_DUE -DARDUINO_ARCH_SAM -D__SAM3X8E__ -DUSB_VID=0x2341 -DUSB_PID=0x003e -DUSBCON  -I"C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/Wire/src" -I"C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/SPI/src" -I"C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/SPI/src/utility" -I"C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries/Wire/src/utility" -I"C:/Program Files (x86)/Arduino/libraries" -I"C:/Program Files (x86)/Arduino/hardware/arduino/avr/libraries" -I"C:/Users/hodgkinr/Documents/Arduino/libraries" -I"C:/Program Files (x86)/Arduino/hardware/arduino/avr/cores/arduino" -I"C:/Program Files (x86)/Arduino/hardware/arduino/avr/variants/standard" -I"../../../IMU_SpinModule" -I"C:/Program Files (x86)/Arduino/hardware/tools/avr/avr/include" -I"C:/Program Files (x86)/Arduino/hardware/tools/avr//avr/include/avr" -I"C:/Program Files (x86)/Arduino/hardware/tools/avr/lib/gcc/avr/4.8.1/include"  -O1 -ffunction-sections -fno-rtti -fno-exceptions -mlong-calls -g3 -Wall -mcpu=cortex-m3 -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker : 5.3.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -mthumb -Wl,-Map="IMU_SpinModule.map" -Wl,--start-group -lm -l:SPI.cpp -l:Wire.cpp  -Wl,--end-group -L"../cmsis/linkerScripts" -L"Z:\Development\2017 Spin Modules\Arduino,LSM9DS1,XBEE\libraries\LSM9DS1\src" -L"../../libraries" -L"C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\SPI" -L"C:\Program Files (x86)\Arduino\hardware\arduino\avr\libraries\Wire"  -Wl,--gc-sections -mcpu=cortex-m3 -Tsamd21e18a_flash.ld  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O binary "IMU_SpinModule.elf" "IMU_SpinModule.bin"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "IMU_SpinModule.elf" "IMU_SpinModule.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O binary "IMU_SpinModule.elf" "IMU_SpinModule.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objdump.exe" -h -S "IMU_SpinModule.elf" > "IMU_SpinModule.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "IMU_SpinModule.elf" "IMU_SpinModule.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-size.exe" "IMU_SpinModule.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "IMU_SpinModule.elf" "IMU_SpinModule.a" "IMU_SpinModule.hex" "IMU_SpinModule.bin" "IMU_SpinModule.lss" "IMU_SpinModule.eep" "IMU_SpinModule.map" "IMU_SpinModule.srec"
	