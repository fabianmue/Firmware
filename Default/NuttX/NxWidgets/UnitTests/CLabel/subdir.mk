################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CLabel/clabel_main.cxx \
../NuttX/NxWidgets/UnitTests/CLabel/clabeltest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CLabel/clabel_main.o \
./NuttX/NxWidgets/UnitTests/CLabel/clabeltest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CLabel/clabel_main.d \
./NuttX/NxWidgets/UnitTests/CLabel/clabeltest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CLabel/%.o: ../NuttX/NxWidgets/UnitTests/CLabel/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


