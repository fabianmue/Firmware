################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/dataman/dataman.c 

OBJS += \
./src/modules/dataman/dataman.o 

C_DEPS += \
./src/modules/dataman/dataman.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/dataman/%.o: ../src/modules/dataman/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


