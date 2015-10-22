################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/parser_200WX/gps_simulator.c \
../src/modules/parser_200WX/parser_200WX.c \
../src/modules/parser_200WX/utilities.c \
../src/modules/parser_200WX/weather_station_utility.c 

OBJS += \
./src/modules/parser_200WX/gps_simulator.o \
./src/modules/parser_200WX/parser_200WX.o \
./src/modules/parser_200WX/utilities.o \
./src/modules/parser_200WX/weather_station_utility.o 

C_DEPS += \
./src/modules/parser_200WX/gps_simulator.d \
./src/modules/parser_200WX/parser_200WX.d \
./src/modules/parser_200WX/utilities.d \
./src/modules/parser_200WX/weather_station_utility.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/parser_200WX/%.o: ../src/modules/parser_200WX/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


