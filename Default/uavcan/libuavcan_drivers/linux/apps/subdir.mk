################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan_drivers/linux/apps/test_clock.cpp \
../uavcan/libuavcan_drivers/linux/apps/test_node.cpp \
../uavcan/libuavcan_drivers/linux/apps/test_socket.cpp \
../uavcan/libuavcan_drivers/linux/apps/test_time_sync.cpp \
../uavcan/libuavcan_drivers/linux/apps/uavcan_nodetool.cpp \
../uavcan/libuavcan_drivers/linux/apps/uavcan_status_monitor.cpp 

OBJS += \
./uavcan/libuavcan_drivers/linux/apps/test_clock.o \
./uavcan/libuavcan_drivers/linux/apps/test_node.o \
./uavcan/libuavcan_drivers/linux/apps/test_socket.o \
./uavcan/libuavcan_drivers/linux/apps/test_time_sync.o \
./uavcan/libuavcan_drivers/linux/apps/uavcan_nodetool.o \
./uavcan/libuavcan_drivers/linux/apps/uavcan_status_monitor.o 

CPP_DEPS += \
./uavcan/libuavcan_drivers/linux/apps/test_clock.d \
./uavcan/libuavcan_drivers/linux/apps/test_node.d \
./uavcan/libuavcan_drivers/linux/apps/test_socket.d \
./uavcan/libuavcan_drivers/linux/apps/test_time_sync.d \
./uavcan/libuavcan_drivers/linux/apps/uavcan_nodetool.d \
./uavcan/libuavcan_drivers/linux/apps/uavcan_status_monitor.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan_drivers/linux/apps/%.o: ../uavcan/libuavcan_drivers/linux/apps/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


