################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/src/protocol/uc_data_type_info_provider.cpp \
../uavcan/libuavcan/src/protocol/uc_global_time_sync_master.cpp \
../uavcan/libuavcan/src/protocol/uc_global_time_sync_slave.cpp \
../uavcan/libuavcan/src/protocol/uc_logger.cpp \
../uavcan/libuavcan/src/protocol/uc_network_compat_checker.cpp \
../uavcan/libuavcan/src/protocol/uc_node_status_monitor.cpp \
../uavcan/libuavcan/src/protocol/uc_node_status_provider.cpp \
../uavcan/libuavcan/src/protocol/uc_panic_broadcaster.cpp \
../uavcan/libuavcan/src/protocol/uc_param_server.cpp \
../uavcan/libuavcan/src/protocol/uc_restart_request_server.cpp \
../uavcan/libuavcan/src/protocol/uc_transport_stats_provider.cpp 

O_SRCS += \
../uavcan/libuavcan/src/protocol/uc_data_type_info_provider.cpp.o \
../uavcan/libuavcan/src/protocol/uc_global_time_sync_master.cpp.o \
../uavcan/libuavcan/src/protocol/uc_global_time_sync_slave.cpp.o \
../uavcan/libuavcan/src/protocol/uc_logger.cpp.o \
../uavcan/libuavcan/src/protocol/uc_network_compat_checker.cpp.o \
../uavcan/libuavcan/src/protocol/uc_node_status_monitor.cpp.o \
../uavcan/libuavcan/src/protocol/uc_node_status_provider.cpp.o \
../uavcan/libuavcan/src/protocol/uc_panic_broadcaster.cpp.o \
../uavcan/libuavcan/src/protocol/uc_param_server.cpp.o \
../uavcan/libuavcan/src/protocol/uc_restart_request_server.cpp.o \
../uavcan/libuavcan/src/protocol/uc_transport_stats_provider.cpp.o 

OBJS += \
./uavcan/libuavcan/src/protocol/uc_data_type_info_provider.o \
./uavcan/libuavcan/src/protocol/uc_global_time_sync_master.o \
./uavcan/libuavcan/src/protocol/uc_global_time_sync_slave.o \
./uavcan/libuavcan/src/protocol/uc_logger.o \
./uavcan/libuavcan/src/protocol/uc_network_compat_checker.o \
./uavcan/libuavcan/src/protocol/uc_node_status_monitor.o \
./uavcan/libuavcan/src/protocol/uc_node_status_provider.o \
./uavcan/libuavcan/src/protocol/uc_panic_broadcaster.o \
./uavcan/libuavcan/src/protocol/uc_param_server.o \
./uavcan/libuavcan/src/protocol/uc_restart_request_server.o \
./uavcan/libuavcan/src/protocol/uc_transport_stats_provider.o 

CPP_DEPS += \
./uavcan/libuavcan/src/protocol/uc_data_type_info_provider.d \
./uavcan/libuavcan/src/protocol/uc_global_time_sync_master.d \
./uavcan/libuavcan/src/protocol/uc_global_time_sync_slave.d \
./uavcan/libuavcan/src/protocol/uc_logger.d \
./uavcan/libuavcan/src/protocol/uc_network_compat_checker.d \
./uavcan/libuavcan/src/protocol/uc_node_status_monitor.d \
./uavcan/libuavcan/src/protocol/uc_node_status_provider.d \
./uavcan/libuavcan/src/protocol/uc_panic_broadcaster.d \
./uavcan/libuavcan/src/protocol/uc_param_server.d \
./uavcan/libuavcan/src/protocol/uc_restart_request_server.d \
./uavcan/libuavcan/src/protocol/uc_transport_stats_provider.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/src/protocol/%.o: ../uavcan/libuavcan/src/protocol/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


