################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/fonksiyonlar/led_camera_kontrol.c 

OBJS += \
./Core/Src/fonksiyonlar/led_camera_kontrol.o 

C_DEPS += \
./Core/Src/fonksiyonlar/led_camera_kontrol.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/fonksiyonlar/%.o Core/Src/fonksiyonlar/%.su Core/Src/fonksiyonlar/%.cyclo: ../Core/Src/fonksiyonlar/%.c Core/Src/fonksiyonlar/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Core/Src/fonksiyonlar -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-fonksiyonlar

clean-Core-2f-Src-2f-fonksiyonlar:
	-$(RM) ./Core/Src/fonksiyonlar/led_camera_kontrol.cyclo ./Core/Src/fonksiyonlar/led_camera_kontrol.d ./Core/Src/fonksiyonlar/led_camera_kontrol.o ./Core/Src/fonksiyonlar/led_camera_kontrol.su

.PHONY: clean-Core-2f-Src-2f-fonksiyonlar

