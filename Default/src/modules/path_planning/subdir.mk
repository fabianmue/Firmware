################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/path_planning/path_planning.c \
../src/modules/path_planning/pp_communication_buffer.c \
../src/modules/path_planning/pp_cost_method.c \
../src/modules/path_planning/pp_failsafe.c \
../src/modules/path_planning/pp_gridlines_handler.c \
../src/modules/path_planning/pp_navigation_helper.c \
../src/modules/path_planning/pp_navigation_module.c \
../src/modules/path_planning/pp_navigator.c \
../src/modules/path_planning/pp_parameters.c \
../src/modules/path_planning/pp_polardiagram.c \
../src/modules/path_planning/pp_potentialfield_method.c \
../src/modules/path_planning/pp_send_msg_qgc.c \
../src/modules/path_planning/pp_topics_handler.c 

OBJS += \
./src/modules/path_planning/path_planning.o \
./src/modules/path_planning/pp_communication_buffer.o \
./src/modules/path_planning/pp_cost_method.o \
./src/modules/path_planning/pp_failsafe.o \
./src/modules/path_planning/pp_gridlines_handler.o \
./src/modules/path_planning/pp_navigation_helper.o \
./src/modules/path_planning/pp_navigation_module.o \
./src/modules/path_planning/pp_navigator.o \
./src/modules/path_planning/pp_parameters.o \
./src/modules/path_planning/pp_polardiagram.o \
./src/modules/path_planning/pp_potentialfield_method.o \
./src/modules/path_planning/pp_send_msg_qgc.o \
./src/modules/path_planning/pp_topics_handler.o 

C_DEPS += \
./src/modules/path_planning/path_planning.d \
./src/modules/path_planning/pp_communication_buffer.d \
./src/modules/path_planning/pp_cost_method.d \
./src/modules/path_planning/pp_failsafe.d \
./src/modules/path_planning/pp_gridlines_handler.d \
./src/modules/path_planning/pp_navigation_helper.d \
./src/modules/path_planning/pp_navigation_module.d \
./src/modules/path_planning/pp_navigator.d \
./src/modules/path_planning/pp_parameters.d \
./src/modules/path_planning/pp_polardiagram.d \
./src/modules/path_planning/pp_potentialfield_method.d \
./src/modules/path_planning/pp_send_msg_qgc.d \
./src/modules/path_planning/pp_topics_handler.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/path_planning/%.o: ../src/modules/path_planning/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


