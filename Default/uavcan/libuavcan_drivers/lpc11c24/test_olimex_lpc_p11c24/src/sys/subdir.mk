################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/crt0.c 

CPP_SRCS += \
../uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/board.cpp \
../uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/libstubs.cpp 

OBJS += \
./uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/board.o \
./uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/crt0.o \
./uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/libstubs.o 

C_DEPS += \
./uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/crt0.d 

CPP_DEPS += \
./uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/board.d \
./uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/libstubs.d 


# Each subdirectory must supply rules for building sources it contributes
uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/%.o: ../uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/%.o: ../uavcan/libuavcan_drivers/lpc11c24/test_olimex_lpc_p11c24/src/sys/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


