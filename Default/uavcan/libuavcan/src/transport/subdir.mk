################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/src/transport/uc_can_io.cpp \
../uavcan/libuavcan/src/transport/uc_crc.cpp \
../uavcan/libuavcan/src/transport/uc_dispatcher.cpp \
../uavcan/libuavcan/src/transport/uc_frame.cpp \
../uavcan/libuavcan/src/transport/uc_outgoing_transfer_registry.cpp \
../uavcan/libuavcan/src/transport/uc_transfer.cpp \
../uavcan/libuavcan/src/transport/uc_transfer_buffer.cpp \
../uavcan/libuavcan/src/transport/uc_transfer_listener.cpp \
../uavcan/libuavcan/src/transport/uc_transfer_receiver.cpp \
../uavcan/libuavcan/src/transport/uc_transfer_sender.cpp 

O_SRCS += \
../uavcan/libuavcan/src/transport/uc_can_io.cpp.o \
../uavcan/libuavcan/src/transport/uc_crc.cpp.o \
../uavcan/libuavcan/src/transport/uc_dispatcher.cpp.o \
../uavcan/libuavcan/src/transport/uc_frame.cpp.o \
../uavcan/libuavcan/src/transport/uc_outgoing_transfer_registry.cpp.o \
../uavcan/libuavcan/src/transport/uc_transfer.cpp.o \
../uavcan/libuavcan/src/transport/uc_transfer_buffer.cpp.o \
../uavcan/libuavcan/src/transport/uc_transfer_listener.cpp.o \
../uavcan/libuavcan/src/transport/uc_transfer_receiver.cpp.o \
../uavcan/libuavcan/src/transport/uc_transfer_sender.cpp.o 

OBJS += \
./uavcan/libuavcan/src/transport/uc_can_io.o \
./uavcan/libuavcan/src/transport/uc_crc.o \
./uavcan/libuavcan/src/transport/uc_dispatcher.o \
./uavcan/libuavcan/src/transport/uc_frame.o \
./uavcan/libuavcan/src/transport/uc_outgoing_transfer_registry.o \
./uavcan/libuavcan/src/transport/uc_transfer.o \
./uavcan/libuavcan/src/transport/uc_transfer_buffer.o \
./uavcan/libuavcan/src/transport/uc_transfer_listener.o \
./uavcan/libuavcan/src/transport/uc_transfer_receiver.o \
./uavcan/libuavcan/src/transport/uc_transfer_sender.o 

CPP_DEPS += \
./uavcan/libuavcan/src/transport/uc_can_io.d \
./uavcan/libuavcan/src/transport/uc_crc.d \
./uavcan/libuavcan/src/transport/uc_dispatcher.d \
./uavcan/libuavcan/src/transport/uc_frame.d \
./uavcan/libuavcan/src/transport/uc_outgoing_transfer_registry.d \
./uavcan/libuavcan/src/transport/uc_transfer.d \
./uavcan/libuavcan/src/transport/uc_transfer_buffer.d \
./uavcan/libuavcan/src/transport/uc_transfer_listener.d \
./uavcan/libuavcan/src/transport/uc_transfer_receiver.d \
./uavcan/libuavcan/src/transport/uc_transfer_sender.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/src/transport/%.o: ../uavcan/libuavcan/src/transport/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


