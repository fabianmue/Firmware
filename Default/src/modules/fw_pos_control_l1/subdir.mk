################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/fw_pos_control_l1/fw_pos_control_l1_params.c 

CPP_SRCS += \
../src/modules/fw_pos_control_l1/fw_pos_control_l1_main.cpp \
../src/modules/fw_pos_control_l1/landingslope.cpp 

OBJS += \
./src/modules/fw_pos_control_l1/fw_pos_control_l1_main.o \
./src/modules/fw_pos_control_l1/fw_pos_control_l1_params.o \
./src/modules/fw_pos_control_l1/landingslope.o 

C_DEPS += \
./src/modules/fw_pos_control_l1/fw_pos_control_l1_params.d 

CPP_DEPS += \
./src/modules/fw_pos_control_l1/fw_pos_control_l1_main.d \
./src/modules/fw_pos_control_l1/landingslope.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/fw_pos_control_l1/%.o: ../src/modules/fw_pos_control_l1/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/fw_pos_control_l1/%.o: ../src/modules/fw_pos_control_l1/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


