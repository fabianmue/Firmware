################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/audio/vs1053.c 

OBJS += \
./NuttX/nuttx/drivers/audio/vs1053.o 

C_DEPS += \
./NuttX/nuttx/drivers/audio/vs1053.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/audio/%.o: ../NuttX/nuttx/drivers/audio/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


