################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/mavlink/mavlink_tests/mavlink_ftp_test.cpp \
../src/modules/mavlink/mavlink_tests/mavlink_tests.cpp 

OBJS += \
./src/modules/mavlink/mavlink_tests/mavlink_ftp_test.o \
./src/modules/mavlink/mavlink_tests/mavlink_tests.o 

CPP_DEPS += \
./src/modules/mavlink/mavlink_tests/mavlink_ftp_test.d \
./src/modules/mavlink/mavlink_tests/mavlink_tests.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/mavlink/mavlink_tests/%.o: ../src/modules/mavlink/mavlink_tests/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


