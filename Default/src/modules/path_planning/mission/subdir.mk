################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/path_planning/mission/mission.c 

OBJS += \
./src/modules/path_planning/mission/mission.o 

C_DEPS += \
./src/modules/path_planning/mission/mission.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/path_planning/mission/%.o: ../src/modules/path_planning/mission/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


