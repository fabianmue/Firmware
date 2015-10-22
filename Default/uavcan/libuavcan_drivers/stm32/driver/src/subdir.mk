################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_can.cpp \
../uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_clock.cpp \
../uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_thread.cpp 

O_SRCS += \
../uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_can.cpp.o \
../uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_clock.cpp.o \
../uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_thread.cpp.o 

OBJS += \
./uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_can.o \
./uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_clock.o \
./uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_thread.o 

CPP_DEPS += \
./uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_can.d \
./uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_clock.d \
./uavcan/libuavcan_drivers/stm32/driver/src/uc_stm32_thread.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan_drivers/stm32/driver/src/%.o: ../uavcan/libuavcan_drivers/stm32/driver/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


