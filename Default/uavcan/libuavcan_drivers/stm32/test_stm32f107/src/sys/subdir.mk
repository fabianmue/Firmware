################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../uavcan/libuavcan_drivers/stm32/test_stm32f107/src/sys/board.c 

OBJS += \
./uavcan/libuavcan_drivers/stm32/test_stm32f107/src/sys/board.o 

C_DEPS += \
./uavcan/libuavcan_drivers/stm32/test_stm32f107/src/sys/board.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan_drivers/stm32/test_stm32f107/src/sys/%.o: ../uavcan/libuavcan_drivers/stm32/test_stm32f107/src/sys/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


