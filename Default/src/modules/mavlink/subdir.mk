################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/mavlink/mavlink.c 

CPP_SRCS += \
../src/modules/mavlink/mavlink_ftp.cpp \
../src/modules/mavlink/mavlink_main.cpp \
../src/modules/mavlink/mavlink_messages.cpp \
../src/modules/mavlink/mavlink_mission.cpp \
../src/modules/mavlink/mavlink_orb_subscription.cpp \
../src/modules/mavlink/mavlink_parameters.cpp \
../src/modules/mavlink/mavlink_rate_limiter.cpp \
../src/modules/mavlink/mavlink_receiver.cpp \
../src/modules/mavlink/mavlink_stream.cpp 

OBJS += \
./src/modules/mavlink/mavlink.o \
./src/modules/mavlink/mavlink_ftp.o \
./src/modules/mavlink/mavlink_main.o \
./src/modules/mavlink/mavlink_messages.o \
./src/modules/mavlink/mavlink_mission.o \
./src/modules/mavlink/mavlink_orb_subscription.o \
./src/modules/mavlink/mavlink_parameters.o \
./src/modules/mavlink/mavlink_rate_limiter.o \
./src/modules/mavlink/mavlink_receiver.o \
./src/modules/mavlink/mavlink_stream.o 

C_DEPS += \
./src/modules/mavlink/mavlink.d 

CPP_DEPS += \
./src/modules/mavlink/mavlink_ftp.d \
./src/modules/mavlink/mavlink_main.d \
./src/modules/mavlink/mavlink_messages.d \
./src/modules/mavlink/mavlink_mission.d \
./src/modules/mavlink/mavlink_orb_subscription.d \
./src/modules/mavlink/mavlink_parameters.d \
./src/modules/mavlink/mavlink_rate_limiter.d \
./src/modules/mavlink/mavlink_receiver.d \
./src/modules/mavlink/mavlink_stream.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/mavlink/%.o: ../src/modules/mavlink/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/mavlink/%.o: ../src/modules/mavlink/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


