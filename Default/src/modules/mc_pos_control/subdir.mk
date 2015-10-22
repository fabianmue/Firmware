################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/mc_pos_control/mc_pos_control_params.c 

CPP_SRCS += \
../src/modules/mc_pos_control/mc_pos_control_main.cpp 

OBJS += \
./src/modules/mc_pos_control/mc_pos_control_main.o \
./src/modules/mc_pos_control/mc_pos_control_params.o 

C_DEPS += \
./src/modules/mc_pos_control/mc_pos_control_params.d 

CPP_DEPS += \
./src/modules/mc_pos_control/mc_pos_control_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/mc_pos_control/%.o: ../src/modules/mc_pos_control/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/mc_pos_control/%.o: ../src/modules/mc_pos_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


