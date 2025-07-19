################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Library/src/STLogo.c \
../Library/src/font12.c \
../Library/src/font16.c \
../Library/src/font20.c \
../Library/src/font24.c \
../Library/src/font8.c \
../Library/src/ili9341.c 

OBJS += \
./Library/src/STLogo.o \
./Library/src/font12.o \
./Library/src/font16.o \
./Library/src/font20.o \
./Library/src/font24.o \
./Library/src/font8.o \
./Library/src/ili9341.o 

C_DEPS += \
./Library/src/STLogo.d \
./Library/src/font12.d \
./Library/src/font16.d \
./Library/src/font20.d \
./Library/src/font24.d \
./Library/src/font8.d \
./Library/src/ili9341.d 


# Each subdirectory must supply rules for building sources it contributes
Library/src/%.o Library/src/%.su Library/src/%.cyclo: ../Library/src/%.c Library/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.18.1/CE103_projectt/Library/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Library-2f-src

clean-Library-2f-src:
	-$(RM) ./Library/src/STLogo.cyclo ./Library/src/STLogo.d ./Library/src/STLogo.o ./Library/src/STLogo.su ./Library/src/font12.cyclo ./Library/src/font12.d ./Library/src/font12.o ./Library/src/font12.su ./Library/src/font16.cyclo ./Library/src/font16.d ./Library/src/font16.o ./Library/src/font16.su ./Library/src/font20.cyclo ./Library/src/font20.d ./Library/src/font20.o ./Library/src/font20.su ./Library/src/font24.cyclo ./Library/src/font24.d ./Library/src/font24.o ./Library/src/font24.su ./Library/src/font8.cyclo ./Library/src/font8.d ./Library/src/font8.o ./Library/src/font8.su ./Library/src/ili9341.cyclo ./Library/src/ili9341.d ./Library/src/ili9341.o ./Library/src/ili9341.su

.PHONY: clean-Library-2f-src

