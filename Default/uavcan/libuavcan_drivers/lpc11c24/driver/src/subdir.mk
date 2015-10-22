################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan_drivers/lpc11c24/driver/src/can.cpp \
../uavcan/libuavcan_drivers/lpc11c24/driver/src/clock.cpp 

OBJS += \
./uavcan/libuavcan_drivers/lpc11c24/driver/src/can.o \
./uavcan/libuavcan_drivers/lpc11c24/driver/src/clock.o 

CPP_DEPS += \
./uavcan/libuavcan_drivers/lpc11c24/driver/src/can.d \
./uavcan/libuavcan_drivers/lpc11c24/driver/src/clock.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan_drivers/lpc11c24/driver/src/%.o: ../uavcan/libuavcan_drivers/lpc11c24/driver/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


