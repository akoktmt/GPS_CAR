################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SteeringHandle/Steering.c 

OBJS += \
./SteeringHandle/Steering.o 

C_DEPS += \
./SteeringHandle/Steering.d 


# Each subdirectory must supply rules for building sources it contributes
SteeringHandle/%.o SteeringHandle/%.su SteeringHandle/%.cyclo: ../SteeringHandle/%.c SteeringHandle/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../EncoderVelocity -I../Motor -I../Motor_Driver -I"../PID Control" -I../BNO -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../CAN_Handle -I../SteeringHandle -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-SteeringHandle

clean-SteeringHandle:
	-$(RM) ./SteeringHandle/Steering.cyclo ./SteeringHandle/Steering.d ./SteeringHandle/Steering.o ./SteeringHandle/Steering.su

.PHONY: clean-SteeringHandle

