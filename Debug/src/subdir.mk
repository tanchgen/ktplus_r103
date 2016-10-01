################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/BlinkLed.c \
../src/_write.c \
../src/buffer.c \
../src/can.c \
../src/flow.c \
../src/main.c \
../src/onewire.c \
../src/stm32f10x_it.c \
../src/time.c 

OBJS += \
./src/BlinkLed.o \
./src/_write.o \
./src/buffer.o \
./src/can.o \
./src/flow.o \
./src/main.o \
./src/onewire.o \
./src/stm32f10x_it.o \
./src/time.o 

C_DEPS += \
./src/BlinkLed.d \
./src/_write.d \
./src/buffer.d \
./src/can.d \
./src/flow.d \
./src/main.d \
./src/onewire.d \
./src/stm32f10x_it.d \
./src/time.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -DCAN_TEST=0 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1-stdperiph" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


