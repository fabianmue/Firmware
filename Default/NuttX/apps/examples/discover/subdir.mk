################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/discover/discover_main.c 

OBJS += \
./NuttX/apps/examples/discover/discover_main.o 

C_DEPS += \
./NuttX/apps/examples/discover/discover_main.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/discover/%.o: ../NuttX/apps/examples/discover/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


