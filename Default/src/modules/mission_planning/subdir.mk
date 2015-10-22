################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/mission_planning/mission_planning.c \
../src/modules/mission_planning/mp_mission.c \
../src/modules/mission_planning/mp_station_keeping.c 

CPP_SRCS += \
../src/modules/mission_planning/mp_readSD.cpp 

OBJS += \
./src/modules/mission_planning/mission_planning.o \
./src/modules/mission_planning/mp_mission.o \
./src/modules/mission_planning/mp_readSD.o \
./src/modules/mission_planning/mp_station_keeping.o 

C_DEPS += \
./src/modules/mission_planning/mission_planning.d \
./src/modules/mission_planning/mp_mission.d \
./src/modules/mission_planning/mp_station_keeping.d 

CPP_DEPS += \
./src/modules/mission_planning/mp_readSD.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/mission_planning/%.o: ../src/modules/mission_planning/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/mission_planning/%.o: ../src/modules/mission_planning/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


