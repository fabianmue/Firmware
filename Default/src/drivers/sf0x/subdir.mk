################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/sf0x/sf0x.cpp \
../src/drivers/sf0x/sf0x_parser.cpp 

OBJS += \
./src/drivers/sf0x/sf0x.o \
./src/drivers/sf0x/sf0x_parser.o 

CPP_DEPS += \
./src/drivers/sf0x/sf0x.d \
./src/drivers/sf0x/sf0x_parser.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/sf0x/%.o: ../src/drivers/sf0x/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


