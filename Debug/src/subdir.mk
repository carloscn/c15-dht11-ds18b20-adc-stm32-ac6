################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/adc.c \
../src/delay.c \
../src/dht11.c \
../src/ds18b20.c \
../src/main.c \
../src/sys.c \
../src/syscalls.c \
../src/system_stm32f10x.c \
../src/timer.c \
../src/uart.c 

OBJS += \
./src/adc.o \
./src/delay.o \
./src/dht11.o \
./src/ds18b20.o \
./src/main.o \
./src/sys.o \
./src/syscalls.o \
./src/system_stm32f10x.o \
./src/timer.o \
./src/uart.o 

C_DEPS += \
./src/adc.d \
./src/delay.d \
./src/dht11.d \
./src/ds18b20.d \
./src/main.d \
./src/sys.d \
./src/syscalls.d \
./src/system_stm32f10x.d \
./src/timer.d \
./src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103ZETx -DDEBUG -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -I"D:/workspace/AC6/Q15_SMARTSENSOR_F103ZET6_AC6/StdPeriph_Driver/inc" -I"D:/workspace/AC6/Q15_SMARTSENSOR_F103ZET6_AC6/inc" -I"D:/workspace/AC6/Q15_SMARTSENSOR_F103ZET6_AC6/CMSIS/device" -I"D:/workspace/AC6/Q15_SMARTSENSOR_F103ZET6_AC6/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


