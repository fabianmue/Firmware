################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/pirelli_dpl10/src/dummy.c 

OBJS += \
./NuttX/nuttx/configs/pirelli_dpl10/src/dummy.o 

C_DEPS += \
./NuttX/nuttx/configs/pirelli_dpl10/src/dummy.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/pirelli_dpl10/src/%.o: ../NuttX/nuttx/configs/pirelli_dpl10/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


