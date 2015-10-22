################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/test/data_type.cpp \
../uavcan/libuavcan/test/dynamic_memory.cpp \
../uavcan/libuavcan/test/test_main.cpp \
../uavcan/libuavcan/test/time.cpp 

OBJS += \
./uavcan/libuavcan/test/data_type.o \
./uavcan/libuavcan/test/dynamic_memory.o \
./uavcan/libuavcan/test/test_main.o \
./uavcan/libuavcan/test/time.o 

CPP_DEPS += \
./uavcan/libuavcan/test/data_type.d \
./uavcan/libuavcan/test/dynamic_memory.d \
./uavcan/libuavcan/test/test_main.d \
./uavcan/libuavcan/test/time.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/test/%.o: ../uavcan/libuavcan/test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


