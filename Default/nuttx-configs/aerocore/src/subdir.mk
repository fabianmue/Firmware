################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../nuttx-configs/aerocore/src/empty.c 

OBJS += \
./nuttx-configs/aerocore/src/empty.o 

C_DEPS += \
./nuttx-configs/aerocore/src/empty.d 


# Each subdirectory must supply rules for building sources it contributes
nuttx-configs/aerocore/src/%.o: ../nuttx-configs/aerocore/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


