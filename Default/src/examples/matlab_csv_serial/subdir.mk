################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/examples/matlab_csv_serial/matlab_csv_serial.c 

OBJS += \
./src/examples/matlab_csv_serial/matlab_csv_serial.o 

C_DEPS += \
./src/examples/matlab_csv_serial/matlab_csv_serial.d 


# Each subdirectory must supply rules for building sources it contributes
src/examples/matlab_csv_serial/%.o: ../src/examples/matlab_csv_serial/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


