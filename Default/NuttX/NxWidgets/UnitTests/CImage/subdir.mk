################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/UnitTests/CImage/cimage_main.cxx \
../NuttX/NxWidgets/UnitTests/CImage/cimagetest.cxx 

OBJS += \
./NuttX/NxWidgets/UnitTests/CImage/cimage_main.o \
./NuttX/NxWidgets/UnitTests/CImage/cimagetest.o 

CXX_DEPS += \
./NuttX/NxWidgets/UnitTests/CImage/cimage_main.d \
./NuttX/NxWidgets/UnitTests/CImage/cimagetest.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/UnitTests/CImage/%.o: ../NuttX/NxWidgets/UnitTests/CImage/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


