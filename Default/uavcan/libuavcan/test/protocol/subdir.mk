################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/test/protocol/data_type_info_provider.cpp \
../uavcan/libuavcan/test/protocol/global_time_sync_master.cpp \
../uavcan/libuavcan/test/protocol/global_time_sync_slave.cpp \
../uavcan/libuavcan/test/protocol/logger.cpp \
../uavcan/libuavcan/test/protocol/network_compat_checker.cpp \
../uavcan/libuavcan/test/protocol/node_status_monitor.cpp \
../uavcan/libuavcan/test/protocol/node_status_provider.cpp \
../uavcan/libuavcan/test/protocol/panic_broadcaster.cpp \
../uavcan/libuavcan/test/protocol/panic_listener.cpp \
../uavcan/libuavcan/test/protocol/param_server.cpp \
../uavcan/libuavcan/test/protocol/restart_request_server.cpp \
../uavcan/libuavcan/test/protocol/transport_stats_provider.cpp 

OBJS += \
./uavcan/libuavcan/test/protocol/data_type_info_provider.o \
./uavcan/libuavcan/test/protocol/global_time_sync_master.o \
./uavcan/libuavcan/test/protocol/global_time_sync_slave.o \
./uavcan/libuavcan/test/protocol/logger.o \
./uavcan/libuavcan/test/protocol/network_compat_checker.o \
./uavcan/libuavcan/test/protocol/node_status_monitor.o \
./uavcan/libuavcan/test/protocol/node_status_provider.o \
./uavcan/libuavcan/test/protocol/panic_broadcaster.o \
./uavcan/libuavcan/test/protocol/panic_listener.o \
./uavcan/libuavcan/test/protocol/param_server.o \
./uavcan/libuavcan/test/protocol/restart_request_server.o \
./uavcan/libuavcan/test/protocol/transport_stats_provider.o 

CPP_DEPS += \
./uavcan/libuavcan/test/protocol/data_type_info_provider.d \
./uavcan/libuavcan/test/protocol/global_time_sync_master.d \
./uavcan/libuavcan/test/protocol/global_time_sync_slave.d \
./uavcan/libuavcan/test/protocol/logger.d \
./uavcan/libuavcan/test/protocol/network_compat_checker.d \
./uavcan/libuavcan/test/protocol/node_status_monitor.d \
./uavcan/libuavcan/test/protocol/node_status_provider.d \
./uavcan/libuavcan/test/protocol/panic_broadcaster.d \
./uavcan/libuavcan/test/protocol/panic_listener.d \
./uavcan/libuavcan/test/protocol/param_server.d \
./uavcan/libuavcan/test/protocol/restart_request_server.d \
./uavcan/libuavcan/test/protocol/transport_stats_provider.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/test/protocol/%.o: ../uavcan/libuavcan/test/protocol/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


