################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CSliderVertical/cslidervertical_main.cxx \
../NuttX/NxWidgets/UnitTests/CSliderVertical/csliderverticaltest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CSliderVertical/cslidervertical_main.o \
./NuttX/NxWidgets/UnitTests/CSliderVertical/csliderverticaltest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CSliderVertical/cslidervertical_main.d \
./NuttX/NxWidgets/UnitTests/CSliderVertical/csliderverticaltest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CSliderVertical/%.o: ../NuttX/NxWidgets/UnitTests/CSliderVertical/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


