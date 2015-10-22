################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/uavcan/sensors/baro.cpp \
../src/modules/uavcan/sensors/gnss.cpp \
../src/modules/uavcan/sensors/mag.cpp \
../src/modules/uavcan/sensors/sensor_bridge.cpp 

OBJS += \
./src/modules/uavcan/sensors/baro.o \
./src/modules/uavcan/sensors/gnss.o \
./src/modules/uavcan/sensors/mag.o \
./src/modules/uavcan/sensors/sensor_bridge.o 

CPP_DEPS += \
./src/modules/uavcan/sensors/baro.d \
./src/modules/uavcan/sensors/gnss.d \
./src/modules/uavcan/sensors/mag.d \
./src/modules/uavcan/sensors/sensor_bridge.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/uavcan/sensors/%.o: ../src/modules/uavcan/sensors/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


