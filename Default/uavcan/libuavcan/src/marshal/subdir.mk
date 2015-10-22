################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/src/marshal/uc_bit_array_copy.cpp \
../uavcan/libuavcan/src/marshal/uc_bit_stream.cpp \
../uavcan/libuavcan/src/marshal/uc_float_spec.cpp \
../uavcan/libuavcan/src/marshal/uc_scalar_codec.cpp 

O_SRCS += \
../uavcan/libuavcan/src/marshal/uc_bit_array_copy.cpp.o \
../uavcan/libuavcan/src/marshal/uc_bit_stream.cpp.o \
../uavcan/libuavcan/src/marshal/uc_float_spec.cpp.o \
../uavcan/libuavcan/src/marshal/uc_scalar_codec.cpp.o 

OBJS += \
./uavcan/libuavcan/src/marshal/uc_bit_array_copy.o \
./uavcan/libuavcan/src/marshal/uc_bit_stream.o \
./uavcan/libuavcan/src/marshal/uc_float_spec.o \
./uavcan/libuavcan/src/marshal/uc_scalar_codec.o 

CPP_DEPS += \
./uavcan/libuavcan/src/marshal/uc_bit_array_copy.d \
./uavcan/libuavcan/src/marshal/uc_bit_stream.d \
./uavcan/libuavcan/src/marshal/uc_float_spec.d \
./uavcan/libuavcan/src/marshal/uc_scalar_codec.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/src/marshal/%.o: ../uavcan/libuavcan/src/marshal/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


