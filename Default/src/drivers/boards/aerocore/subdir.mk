################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers/boards/aerocore/aerocore_init.c \
../src/drivers/boards/aerocore/aerocore_led.c \
../src/drivers/boards/aerocore/aerocore_pwm_servo.c \
../src/drivers/boards/aerocore/aerocore_spi.c 

OBJS += \
./src/drivers/boards/aerocore/aerocore_init.o \
./src/drivers/boards/aerocore/aerocore_led.o \
./src/drivers/boards/aerocore/aerocore_pwm_servo.o \
./src/drivers/boards/aerocore/aerocore_spi.o 

C_DEPS += \
./src/drivers/boards/aerocore/aerocore_init.d \
./src/drivers/boards/aerocore/aerocore_led.d \
./src/drivers/boards/aerocore/aerocore_pwm_servo.d \
./src/drivers/boards/aerocore/aerocore_spi.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/boards/aerocore/%.o: ../src/drivers/boards/aerocore/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


