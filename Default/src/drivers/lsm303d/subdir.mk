################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/lsm303d/lsm303d.cpp 

OBJS += \
./src/drivers/lsm303d/lsm303d.o 

CPP_DEPS += \
./src/drivers/lsm303d/lsm303d.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/lsm303d/%.o: ../src/drivers/lsm303d/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


