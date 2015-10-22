################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/test/transport/crc.cpp \
../uavcan/libuavcan/test/transport/dispatcher.cpp \
../uavcan/libuavcan/test/transport/frame.cpp \
../uavcan/libuavcan/test/transport/incoming_transfer.cpp \
../uavcan/libuavcan/test/transport/outgoing_transfer_registry.cpp \
../uavcan/libuavcan/test/transport/transfer.cpp \
../uavcan/libuavcan/test/transport/transfer_buffer.cpp \
../uavcan/libuavcan/test/transport/transfer_listener.cpp \
../uavcan/libuavcan/test/transport/transfer_receiver.cpp \
../uavcan/libuavcan/test/transport/transfer_sender.cpp \
../uavcan/libuavcan/test/transport/transfer_test_helpers.cpp 

OBJS += \
./uavcan/libuavcan/test/transport/crc.o \
./uavcan/libuavcan/test/transport/dispatcher.o \
./uavcan/libuavcan/test/transport/frame.o \
./uavcan/libuavcan/test/transport/incoming_transfer.o \
./uavcan/libuavcan/test/transport/outgoing_transfer_registry.o \
./uavcan/libuavcan/test/transport/transfer.o \
./uavcan/libuavcan/test/transport/transfer_buffer.o \
./uavcan/libuavcan/test/transport/transfer_listener.o \
./uavcan/libuavcan/test/transport/transfer_receiver.o \
./uavcan/libuavcan/test/transport/transfer_sender.o \
./uavcan/libuavcan/test/transport/transfer_test_helpers.o 

CPP_DEPS += \
./uavcan/libuavcan/test/transport/crc.d \
./uavcan/libuavcan/test/transport/dispatcher.d \
./uavcan/libuavcan/test/transport/frame.d \
./uavcan/libuavcan/test/transport/incoming_transfer.d \
./uavcan/libuavcan/test/transport/outgoing_transfer_registry.d \
./uavcan/libuavcan/test/transport/transfer.d \
./uavcan/libuavcan/test/transport/transfer_buffer.d \
./uavcan/libuavcan/test/transport/transfer_listener.d \
./uavcan/libuavcan/test/transport/transfer_receiver.d \
./uavcan/libuavcan/test/transport/transfer_sender.d \
./uavcan/libuavcan/test/transport/transfer_test_helpers.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/test/transport/%.o: ../uavcan/libuavcan/test/transport/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


