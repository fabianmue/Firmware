################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/lib/launchdetection/launchdetection_params.c 

CPP_SRCS += \
../src/lib/launchdetection/CatapultLaunchMethod.cpp \
../src/lib/launchdetection/LaunchDetector.cpp 

OBJS += \
./src/lib/launchdetection/CatapultLaunchMethod.o \
./src/lib/launchdetection/LaunchDetector.o \
./src/lib/launchdetection/launchdetection_params.o 

C_DEPS += \
./src/lib/launchdetection/launchdetection_params.d 

CPP_DEPS += \
./src/lib/launchdetection/CatapultLaunchMethod.d \
./src/lib/launchdetection/LaunchDetector.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/launchdetection/%.o: ../src/lib/launchdetection/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/lib/launchdetection/%.o: ../src/lib/launchdetection/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


