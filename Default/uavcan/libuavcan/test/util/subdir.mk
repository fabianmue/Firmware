################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/test/util/comparison.cpp \
../uavcan/libuavcan/test/util/lazy_constructor.cpp \
../uavcan/libuavcan/test/util/linked_list.cpp \
../uavcan/libuavcan/test/util/map.cpp \
../uavcan/libuavcan/test/util/templates.cpp 

OBJS += \
./uavcan/libuavcan/test/util/comparison.o \
./uavcan/libuavcan/test/util/lazy_constructor.o \
./uavcan/libuavcan/test/util/linked_list.o \
./uavcan/libuavcan/test/util/map.o \
./uavcan/libuavcan/test/util/templates.o 

CPP_DEPS += \
./uavcan/libuavcan/test/util/comparison.d \
./uavcan/libuavcan/test/util/lazy_constructor.d \
./uavcan/libuavcan/test/util/linked_list.d \
./uavcan/libuavcan/test/util/map.d \
./uavcan/libuavcan/test/util/templates.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/test/util/%.o: ../uavcan/libuavcan/test/util/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


