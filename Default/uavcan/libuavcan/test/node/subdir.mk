################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/test/node/global_data_type_registry.cpp \
../uavcan/libuavcan/test/node/node.cpp \
../uavcan/libuavcan/test/node/publisher.cpp \
../uavcan/libuavcan/test/node/scheduler.cpp \
../uavcan/libuavcan/test/node/service_client.cpp \
../uavcan/libuavcan/test/node/service_server.cpp \
../uavcan/libuavcan/test/node/subscriber.cpp 

OBJS += \
./uavcan/libuavcan/test/node/global_data_type_registry.o \
./uavcan/libuavcan/test/node/node.o \
./uavcan/libuavcan/test/node/publisher.o \
./uavcan/libuavcan/test/node/scheduler.o \
./uavcan/libuavcan/test/node/service_client.o \
./uavcan/libuavcan/test/node/service_server.o \
./uavcan/libuavcan/test/node/subscriber.o 

CPP_DEPS += \
./uavcan/libuavcan/test/node/global_data_type_registry.d \
./uavcan/libuavcan/test/node/node.d \
./uavcan/libuavcan/test/node/publisher.d \
./uavcan/libuavcan/test/node/scheduler.d \
./uavcan/libuavcan/test/node/service_client.d \
./uavcan/libuavcan/test/node/service_server.d \
./uavcan/libuavcan/test/node/subscriber.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/test/node/%.o: ../uavcan/libuavcan/test/node/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


