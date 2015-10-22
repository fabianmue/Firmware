################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../uavcan/libuavcan/src/driver/uc_can.cpp 

O_SRCS += \
../uavcan/libuavcan/src/driver/uc_can.cpp.o 

OBJS += \
./uavcan/libuavcan/src/driver/uc_can.o 

CPP_DEPS += \
./uavcan/libuavcan/src/driver/uc_can.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan/src/driver/%.o: ../uavcan/libuavcan/src/driver/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


