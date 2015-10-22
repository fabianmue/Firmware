################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/test/transport/can/can_driver.cpp \
../uavcan/libuavcan/test/transport/can/iface_mock.cpp \
../uavcan/libuavcan/test/transport/can/io.cpp \
../uavcan/libuavcan/test/transport/can/tx_queue.cpp 

OBJS += \
./uavcan/libuavcan/test/transport/can/can_driver.o \
./uavcan/libuavcan/test/transport/can/iface_mock.o \
./uavcan/libuavcan/test/transport/can/io.o \
./uavcan/libuavcan/test/transport/can/tx_queue.o 

CPP_DEPS += \
./uavcan/libuavcan/test/transport/can/can_driver.d \
./uavcan/libuavcan/test/transport/can/iface_mock.d \
./uavcan/libuavcan/test/transport/can/io.d \
./uavcan/libuavcan/test/transport/can/tx_queue.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/test/transport/can/%.o: ../uavcan/libuavcan/test/transport/can/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


