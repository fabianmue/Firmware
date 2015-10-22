################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/tools/coverity_scan_model.cpp 

OBJS += \
./uavcan/libuavcan/tools/coverity_scan_model.o 

CPP_DEPS += \
./uavcan/libuavcan/tools/coverity_scan_model.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/tools/%.o: ../uavcan/libuavcan/tools/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


