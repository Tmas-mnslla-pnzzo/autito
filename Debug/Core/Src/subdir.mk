################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ESP01.c \
../Core/Src/hcsr04.c \
../Core/Src/linefollower.c \
../Core/Src/main.c \
../Core/Src/protocolo.c \
../Core/Src/sg90.c \
../Core/Src/soft_i2c.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/tcrt5000.c 

OBJS += \
./Core/Src/ESP01.o \
./Core/Src/hcsr04.o \
./Core/Src/linefollower.o \
./Core/Src/main.o \
./Core/Src/protocolo.o \
./Core/Src/sg90.o \
./Core/Src/soft_i2c.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/tcrt5000.o 

C_DEPS += \
./Core/Src/ESP01.d \
./Core/Src/hcsr04.d \
./Core/Src/linefollower.d \
./Core/Src/main.d \
./Core/Src/protocolo.d \
./Core/Src/sg90.d \
./Core/Src/soft_i2c.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/tcrt5000.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ESP01.cyclo ./Core/Src/ESP01.d ./Core/Src/ESP01.o ./Core/Src/ESP01.su ./Core/Src/hcsr04.cyclo ./Core/Src/hcsr04.d ./Core/Src/hcsr04.o ./Core/Src/hcsr04.su ./Core/Src/linefollower.cyclo ./Core/Src/linefollower.d ./Core/Src/linefollower.o ./Core/Src/linefollower.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/protocolo.cyclo ./Core/Src/protocolo.d ./Core/Src/protocolo.o ./Core/Src/protocolo.su ./Core/Src/sg90.cyclo ./Core/Src/sg90.d ./Core/Src/sg90.o ./Core/Src/sg90.su ./Core/Src/soft_i2c.cyclo ./Core/Src/soft_i2c.d ./Core/Src/soft_i2c.o ./Core/Src/soft_i2c.su ./Core/Src/ssd1306.cyclo ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/stm32f1xx_hal_msp.cyclo ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.cyclo ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.cyclo ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/tcrt5000.cyclo ./Core/Src/tcrt5000.d ./Core/Src/tcrt5000.o ./Core/Src/tcrt5000.su

.PHONY: clean-Core-2f-Src

