################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/fw_pos_control_l1/mtecs/mTecs_params.c 

CPP_SRCS += \
../src/modules/fw_pos_control_l1/mtecs/limitoverride.cpp \
../src/modules/fw_pos_control_l1/mtecs/mTecs.cpp 

OBJS += \
./src/modules/fw_pos_control_l1/mtecs/limitoverride.o \
./src/modules/fw_pos_control_l1/mtecs/mTecs.o \
./src/modules/fw_pos_control_l1/mtecs/mTecs_params.o 

C_DEPS += \
./src/modules/fw_pos_control_l1/mtecs/mTecs_params.d 

CPP_DEPS += \
./src/modules/fw_pos_control_l1/mtecs/limitoverride.d \
./src/modules/fw_pos_control_l1/mtecs/mTecs.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/fw_pos_control_l1/mtecs/%.o: ../src/modules/fw_pos_control_l1/mtecs/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/fw_pos_control_l1/mtecs/%.o: ../src/modules/fw_pos_control_l1/mtecs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


