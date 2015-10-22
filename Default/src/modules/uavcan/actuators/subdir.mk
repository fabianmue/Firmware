################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/uavcan/actuators/esc.cpp 

OBJS += \
./src/modules/uavcan/actuators/esc.o 

CPP_DEPS += \
./src/modules/uavcan/actuators/esc.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/uavcan/actuators/%.o: ../src/modules/uavcan/actuators/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


