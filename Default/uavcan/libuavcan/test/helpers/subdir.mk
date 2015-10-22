################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/test/helpers/component_status_manager.cpp 

OBJS += \
./uavcan/libuavcan/test/helpers/component_status_manager.o 

CPP_DEPS += \
./uavcan/libuavcan/test/helpers/component_status_manager.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/test/helpers/%.o: ../uavcan/libuavcan/test/helpers/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


