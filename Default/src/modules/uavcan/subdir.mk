################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/uavcan/uavcan_params.c 

CPP_SRCS += \
../src/modules/uavcan/uavcan_clock.cpp \
../src/modules/uavcan/uavcan_main.cpp 

OBJS += \
./src/modules/uavcan/uavcan_clock.o \
./src/modules/uavcan/uavcan_main.o \
./src/modules/uavcan/uavcan_params.o 

C_DEPS += \
./src/modules/uavcan/uavcan_params.d 

CPP_DEPS += \
./src/modules/uavcan/uavcan_clock.d \
./src/modules/uavcan/uavcan_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/uavcan/%.o: ../src/modules/uavcan/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/uavcan/%.o: ../src/modules/uavcan/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


