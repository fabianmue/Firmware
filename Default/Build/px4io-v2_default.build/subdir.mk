################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Build/px4io-v2_default.build/empty.c 

OBJS += \
./Build/px4io-v2_default.build/empty.o 

C_DEPS += \
./Build/px4io-v2_default.build/empty.d 


# Each subdirectory must supply rules for building sources it contributes
Build/px4io-v2_default.build/%.o: ../Build/px4io-v2_default.build/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


