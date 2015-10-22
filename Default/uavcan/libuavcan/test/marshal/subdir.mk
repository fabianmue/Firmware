################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/test/marshal/array.cpp \
../uavcan/libuavcan/test/marshal/bit_stream.cpp \
../uavcan/libuavcan/test/marshal/char_array_formatter.cpp \
../uavcan/libuavcan/test/marshal/float_spec.cpp \
../uavcan/libuavcan/test/marshal/integer_spec.cpp \
../uavcan/libuavcan/test/marshal/scalar_codec.cpp \
../uavcan/libuavcan/test/marshal/type_util.cpp 

OBJS += \
./uavcan/libuavcan/test/marshal/array.o \
./uavcan/libuavcan/test/marshal/bit_stream.o \
./uavcan/libuavcan/test/marshal/char_array_formatter.o \
./uavcan/libuavcan/test/marshal/float_spec.o \
./uavcan/libuavcan/test/marshal/integer_spec.o \
./uavcan/libuavcan/test/marshal/scalar_codec.o \
./uavcan/libuavcan/test/marshal/type_util.o 

CPP_DEPS += \
./uavcan/libuavcan/test/marshal/array.d \
./uavcan/libuavcan/test/marshal/bit_stream.d \
./uavcan/libuavcan/test/marshal/char_array_formatter.d \
./uavcan/libuavcan/test/marshal/float_spec.d \
./uavcan/libuavcan/test/marshal/integer_spec.d \
./uavcan/libuavcan/test/marshal/scalar_codec.d \
./uavcan/libuavcan/test/marshal/type_util.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/test/marshal/%.o: ../uavcan/libuavcan/test/marshal/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


