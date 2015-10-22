################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/src/uc_data_type.cpp \
../uavcan/libuavcan/src/uc_dynamic_memory.cpp \
../uavcan/libuavcan/src/uc_error.cpp 

O_SRCS += \
../uavcan/libuavcan/src/uc_data_type.cpp.o \
../uavcan/libuavcan/src/uc_dynamic_memory.cpp.o \
../uavcan/libuavcan/src/uc_error.cpp.o 

OBJS += \
./uavcan/libuavcan/src/uc_data_type.o \
./uavcan/libuavcan/src/uc_dynamic_memory.o \
./uavcan/libuavcan/src/uc_error.o 

CPP_DEPS += \
./uavcan/libuavcan/src/uc_data_type.d \
./uavcan/libuavcan/src/uc_dynamic_memory.d \
./uavcan/libuavcan/src/uc_error.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/src/%.o: ../uavcan/libuavcan/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


