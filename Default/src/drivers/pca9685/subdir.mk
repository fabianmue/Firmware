################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/pca9685/pca9685.cpp 

OBJS += \
./src/drivers/pca9685/pca9685.o 

CPP_DEPS += \
./src/drivers/pca9685/pca9685.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/pca9685/%.o: ../src/drivers/pca9685/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


