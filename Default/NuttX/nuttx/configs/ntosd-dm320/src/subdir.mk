################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/ntosd-dm320/src/up_leds.c \
../NuttX/nuttx/configs/ntosd-dm320/src/up_network.c 

OBJS += \
./NuttX/nuttx/configs/ntosd-dm320/src/up_leds.o \
./NuttX/nuttx/configs/ntosd-dm320/src/up_network.o 

C_DEPS += \
./NuttX/nuttx/configs/ntosd-dm320/src/up_leds.d \
./NuttX/nuttx/configs/ntosd-dm320/src/up_network.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/ntosd-dm320/src/%.o: ../NuttX/nuttx/configs/ntosd-dm320/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


