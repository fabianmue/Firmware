################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h10.c \
../src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h15.c \
../src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h20.c \
../src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h25.c \
../src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h30.c 

O_SRCS += \
../src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h20.o \
../src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h30.o 

OBJS += \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h10.o \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h15.o \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h20.o \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h25.o \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h30.o 

C_DEPS += \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h10.d \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h15.d \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h20.d \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h25.d \
./src/modules/autonomous_sailing/mpcForces/mpc_boatTack_h30.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/autonomous_sailing/mpcForces/%.o: ../src/modules/autonomous_sailing/mpcForces/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


