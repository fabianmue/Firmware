################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/lib/rc/st24.c 

OBJS += \
./src/lib/rc/st24.o 

C_DEPS += \
./src/lib/rc/st24.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/rc/%.o: ../src/lib/rc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


