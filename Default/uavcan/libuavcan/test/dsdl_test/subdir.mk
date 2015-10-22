################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/test/dsdl_test/dsdl_const_1.cpp \
../uavcan/libuavcan/test/dsdl_test/dsdl_const_2.cpp \
../uavcan/libuavcan/test/dsdl_test/dsdl_test.cpp \
../uavcan/libuavcan/test/dsdl_test/dsdl_uavcan_compilability.cpp 

OBJS += \
./uavcan/libuavcan/test/dsdl_test/dsdl_const_1.o \
./uavcan/libuavcan/test/dsdl_test/dsdl_const_2.o \
./uavcan/libuavcan/test/dsdl_test/dsdl_test.o \
./uavcan/libuavcan/test/dsdl_test/dsdl_uavcan_compilability.o 

CPP_DEPS += \
./uavcan/libuavcan/test/dsdl_test/dsdl_const_1.d \
./uavcan/libuavcan/test/dsdl_test/dsdl_const_2.d \
./uavcan/libuavcan/test/dsdl_test/dsdl_test.d \
./uavcan/libuavcan/test/dsdl_test/dsdl_uavcan_compilability.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/test/dsdl_test/%.o: ../uavcan/libuavcan/test/dsdl_test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


