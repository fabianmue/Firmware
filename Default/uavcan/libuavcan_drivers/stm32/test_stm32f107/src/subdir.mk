################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan_drivers/stm32/test_stm32f107/src/dummy.cpp \
../uavcan/libuavcan_drivers/stm32/test_stm32f107/src/main.cpp 

OBJS += \
./uavcan/libuavcan_drivers/stm32/test_stm32f107/src/dummy.o \
./uavcan/libuavcan_drivers/stm32/test_stm32f107/src/main.o 

CPP_DEPS += \
./uavcan/libuavcan_drivers/stm32/test_stm32f107/src/dummy.d \
./uavcan/libuavcan_drivers/stm32/test_stm32f107/src/main.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan_drivers/stm32/test_stm32f107/src/%.o: ../uavcan/libuavcan_drivers/stm32/test_stm32f107/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


