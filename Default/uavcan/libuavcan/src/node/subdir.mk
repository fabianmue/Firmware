################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/src/node/uc_generic_publisher.cpp \
../uavcan/libuavcan/src/node/uc_generic_subscriber.cpp \
../uavcan/libuavcan/src/node/uc_global_data_type_registry.cpp \
../uavcan/libuavcan/src/node/uc_scheduler.cpp \
../uavcan/libuavcan/src/node/uc_service_client.cpp \
../uavcan/libuavcan/src/node/uc_timer.cpp 

O_SRCS += \
../uavcan/libuavcan/src/node/uc_generic_publisher.cpp.o \
../uavcan/libuavcan/src/node/uc_generic_subscriber.cpp.o \
../uavcan/libuavcan/src/node/uc_global_data_type_registry.cpp.o \
../uavcan/libuavcan/src/node/uc_scheduler.cpp.o \
../uavcan/libuavcan/src/node/uc_service_client.cpp.o \
../uavcan/libuavcan/src/node/uc_timer.cpp.o 

OBJS += \
./uavcan/libuavcan/src/node/uc_generic_publisher.o \
./uavcan/libuavcan/src/node/uc_generic_subscriber.o \
./uavcan/libuavcan/src/node/uc_global_data_type_registry.o \
./uavcan/libuavcan/src/node/uc_scheduler.o \
./uavcan/libuavcan/src/node/uc_service_client.o \
./uavcan/libuavcan/src/node/uc_timer.o 

CPP_DEPS += \
./uavcan/libuavcan/src/node/uc_generic_publisher.d \
./uavcan/libuavcan/src/node/uc_generic_subscriber.d \
./uavcan/libuavcan/src/node/uc_global_data_type_registry.d \
./uavcan/libuavcan/src/node/uc_scheduler.d \
./uavcan/libuavcan/src/node/uc_service_client.d \
./uavcan/libuavcan/src/node/uc_timer.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/src/node/%.o: ../uavcan/libuavcan/src/node/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


