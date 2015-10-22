################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/ver/ver.c 

OBJS += \
./src/systemcmds/ver/ver.o 

C_DEPS += \
./src/systemcmds/ver/ver.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/ver/%.o: ../src/systemcmds/ver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


