################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/lib/mathlib/math/test/test.cpp 

OBJS += \
./src/lib/mathlib/math/test/test.o 

CPP_DEPS += \
./src/lib/mathlib/math/test/test.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/mathlib/math/test/%.o: ../src/lib/mathlib/math/test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


