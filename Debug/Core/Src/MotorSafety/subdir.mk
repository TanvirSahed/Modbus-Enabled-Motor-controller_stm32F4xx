################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MotorSafety/pwmIF.c \
../Core/Src/MotorSafety/pwm_driver.c 

OBJS += \
./Core/Src/MotorSafety/pwmIF.o \
./Core/Src/MotorSafety/pwm_driver.o 

C_DEPS += \
./Core/Src/MotorSafety/pwmIF.d \
./Core/Src/MotorSafety/pwm_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MotorSafety/%.o Core/Src/MotorSafety/%.su Core/Src/MotorSafety/%.cyclo: ../Core/Src/MotorSafety/%.c Core/Src/MotorSafety/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/wiznet/Inc" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/PID" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/Filters" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/DM542T" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/Encoder" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/StepperMotor" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/BTS7960" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/MotorController" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/MyTimer" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/TimeStamp" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/datalink" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/libmodbus" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/wiznet" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/datalink/Inc" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/datalink/Src" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/libmodbus/Inc" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/libmodbus/Src" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/w25qxx" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/MotorSafety" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-MotorSafety

clean-Core-2f-Src-2f-MotorSafety:
	-$(RM) ./Core/Src/MotorSafety/pwmIF.cyclo ./Core/Src/MotorSafety/pwmIF.d ./Core/Src/MotorSafety/pwmIF.o ./Core/Src/MotorSafety/pwmIF.su ./Core/Src/MotorSafety/pwm_driver.cyclo ./Core/Src/MotorSafety/pwm_driver.d ./Core/Src/MotorSafety/pwm_driver.o ./Core/Src/MotorSafety/pwm_driver.su

.PHONY: clean-Core-2f-Src-2f-MotorSafety

