################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers/boards/px4io-v1/px4io_init.c \
../src/drivers/boards/px4io-v1/px4io_pwm_servo.c 

OBJS += \
./src/drivers/boards/px4io-v1/px4io_init.o \
./src/drivers/boards/px4io-v1/px4io_pwm_servo.o 

C_DEPS += \
./src/drivers/boards/px4io-v1/px4io_init.d \
./src/drivers/boards/px4io-v1/px4io_pwm_servo.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/boards/px4io-v1/%.o: ../src/drivers/boards/px4io-v1/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


