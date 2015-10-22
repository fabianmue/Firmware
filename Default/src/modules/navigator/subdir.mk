################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/navigator/datalinkloss_params.c \
../src/modules/navigator/geofence_params.c \
../src/modules/navigator/gpsfailure_params.c \
../src/modules/navigator/mission_params.c \
../src/modules/navigator/navigator_params.c \
../src/modules/navigator/rcloss_params.c \
../src/modules/navigator/rtl_params.c 

CPP_SRCS += \
../src/modules/navigator/datalinkloss.cpp \
../src/modules/navigator/enginefailure.cpp \
../src/modules/navigator/geofence.cpp \
../src/modules/navigator/gpsfailure.cpp \
../src/modules/navigator/loiter.cpp \
../src/modules/navigator/mission.cpp \
../src/modules/navigator/mission_block.cpp \
../src/modules/navigator/mission_feasibility_checker.cpp \
../src/modules/navigator/navigator_main.cpp \
../src/modules/navigator/navigator_mode.cpp \
../src/modules/navigator/rcloss.cpp \
../src/modules/navigator/rtl.cpp 

OBJS += \
./src/modules/navigator/datalinkloss.o \
./src/modules/navigator/datalinkloss_params.o \
./src/modules/navigator/enginefailure.o \
./src/modules/navigator/geofence.o \
./src/modules/navigator/geofence_params.o \
./src/modules/navigator/gpsfailure.o \
./src/modules/navigator/gpsfailure_params.o \
./src/modules/navigator/loiter.o \
./src/modules/navigator/mission.o \
./src/modules/navigator/mission_block.o \
./src/modules/navigator/mission_feasibility_checker.o \
./src/modules/navigator/mission_params.o \
./src/modules/navigator/navigator_main.o \
./src/modules/navigator/navigator_mode.o \
./src/modules/navigator/navigator_params.o \
./src/modules/navigator/rcloss.o \
./src/modules/navigator/rcloss_params.o \
./src/modules/navigator/rtl.o \
./src/modules/navigator/rtl_params.o 

C_DEPS += \
./src/modules/navigator/datalinkloss_params.d \
./src/modules/navigator/geofence_params.d \
./src/modules/navigator/gpsfailure_params.d \
./src/modules/navigator/mission_params.d \
./src/modules/navigator/navigator_params.d \
./src/modules/navigator/rcloss_params.d \
./src/modules/navigator/rtl_params.d 

CPP_DEPS += \
./src/modules/navigator/datalinkloss.d \
./src/modules/navigator/enginefailure.d \
./src/modules/navigator/geofence.d \
./src/modules/navigator/gpsfailure.d \
./src/modules/navigator/loiter.d \
./src/modules/navigator/mission.d \
./src/modules/navigator/mission_block.d \
./src/modules/navigator/mission_feasibility_checker.d \
./src/modules/navigator/navigator_main.d \
./src/modules/navigator/navigator_mode.d \
./src/modules/navigator/rcloss.d \
./src/modules/navigator/rtl.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/navigator/%.o: ../src/modules/navigator/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/navigator/%.o: ../src/modules/navigator/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


