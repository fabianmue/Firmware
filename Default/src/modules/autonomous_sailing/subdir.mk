################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/autonomous_sailing/autonomous_sailing.c \
../src/modules/autonomous_sailing/controller_data.c \
../src/modules/autonomous_sailing/extremum_sailcontrol.c \
../src/modules/autonomous_sailing/guidance_module.c \
../src/modules/autonomous_sailing/mpc_test_data.c \
../src/modules/autonomous_sailing/parameters.c \
../src/modules/autonomous_sailing/send_msg_qgc.c 

OBJS += \
./src/modules/autonomous_sailing/autonomous_sailing.o \
./src/modules/autonomous_sailing/controller_data.o \
./src/modules/autonomous_sailing/extremum_sailcontrol.o \
./src/modules/autonomous_sailing/guidance_module.o \
./src/modules/autonomous_sailing/mpc_test_data.o \
./src/modules/autonomous_sailing/parameters.o \
./src/modules/autonomous_sailing/send_msg_qgc.o 

C_DEPS += \
./src/modules/autonomous_sailing/autonomous_sailing.d \
./src/modules/autonomous_sailing/controller_data.d \
./src/modules/autonomous_sailing/extremum_sailcontrol.d \
./src/modules/autonomous_sailing/guidance_module.d \
./src/modules/autonomous_sailing/mpc_test_data.d \
./src/modules/autonomous_sailing/parameters.d \
./src/modules/autonomous_sailing/send_msg_qgc.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/autonomous_sailing/%.o: ../src/modules/autonomous_sailing/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


