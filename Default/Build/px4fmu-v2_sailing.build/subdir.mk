################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Build/px4fmu-v2_sailing.build/builtin_commands.c 

O_SRCS += \
../Build/px4fmu-v2_sailing.build/romfs.o 

OBJS += \
./Build/px4fmu-v2_sailing.build/builtin_commands.o 

C_DEPS += \
./Build/px4fmu-v2_sailing.build/builtin_commands.d 


# Each subdirectory must supply rules for building sources it contributes
Build/px4fmu-v2_sailing.build/%.o: ../Build/px4fmu-v2_sailing.build/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


