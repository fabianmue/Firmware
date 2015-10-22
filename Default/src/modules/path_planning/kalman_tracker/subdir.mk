################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/path_planning/kalman_tracker/kt_cog_list.c \
../src/modules/path_planning/kalman_tracker/kt_topic_handler.c \
../src/modules/path_planning/kalman_tracker/kt_track_list.c \
../src/modules/path_planning/kalman_tracker/kt_tracker.c 

OBJS += \
./src/modules/path_planning/kalman_tracker/kt_cog_list.o \
./src/modules/path_planning/kalman_tracker/kt_topic_handler.o \
./src/modules/path_planning/kalman_tracker/kt_track_list.o \
./src/modules/path_planning/kalman_tracker/kt_tracker.o 

C_DEPS += \
./src/modules/path_planning/kalman_tracker/kt_cog_list.d \
./src/modules/path_planning/kalman_tracker/kt_topic_handler.d \
./src/modules/path_planning/kalman_tracker/kt_track_list.d \
./src/modules/path_planning/kalman_tracker/kt_tracker.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/path_planning/kalman_tracker/%.o: ../src/modules/path_planning/kalman_tracker/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


