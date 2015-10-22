################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/ekf_att_pos_estimator/ekf_att_pos_estimator_params.c 

CPP_SRCS += \
../src/modules/ekf_att_pos_estimator/ekf_att_pos_estimator_main.cpp \
../src/modules/ekf_att_pos_estimator/estimator_21states.cpp \
../src/modules/ekf_att_pos_estimator/estimator_23states.cpp \
../src/modules/ekf_att_pos_estimator/estimator_utilities.cpp 

OBJS += \
./src/modules/ekf_att_pos_estimator/ekf_att_pos_estimator_main.o \
./src/modules/ekf_att_pos_estimator/ekf_att_pos_estimator_params.o \
./src/modules/ekf_att_pos_estimator/estimator_21states.o \
./src/modules/ekf_att_pos_estimator/estimator_23states.o \
./src/modules/ekf_att_pos_estimator/estimator_utilities.o 

C_DEPS += \
./src/modules/ekf_att_pos_estimator/ekf_att_pos_estimator_params.d 

CPP_DEPS += \
./src/modules/ekf_att_pos_estimator/ekf_att_pos_estimator_main.d \
./src/modules/ekf_att_pos_estimator/estimator_21states.d \
./src/modules/ekf_att_pos_estimator/estimator_23states.d \
./src/modules/ekf_att_pos_estimator/estimator_utilities.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/ekf_att_pos_estimator/%.o: ../src/modules/ekf_att_pos_estimator/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/ekf_att_pos_estimator/%.o: ../src/modules/ekf_att_pos_estimator/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


