################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/StepperMotor/StepperMotor.c \
../Core/Src/StepperMotor/StepperMotorIf.c 

OBJS += \
./Core/Src/StepperMotor/StepperMotor.o \
./Core/Src/StepperMotor/StepperMotorIf.o 

C_DEPS += \
./Core/Src/StepperMotor/StepperMotor.d \
./Core/Src/StepperMotor/StepperMotorIf.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/StepperMotor/%.o Core/Src/StepperMotor/%.su Core/Src/StepperMotor/%.cyclo: ../Core/Src/StepperMotor/%.c Core/Src/StepperMotor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/wiznet/Inc" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/PID" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/Filters" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/DM542T" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/Encoder" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/StepperMotor" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/BTS7960" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/MotorController" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/MyTimer" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/TimeStamp" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/datalink" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/libmodbus" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/wiznet" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/datalink/Inc" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/datalink/Src" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/libmodbus/Inc" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/libmodbus/Src" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/w25qxx" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/MotorSafety" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-StepperMotor

clean-Core-2f-Src-2f-StepperMotor:
	-$(RM) ./Core/Src/StepperMotor/StepperMotor.cyclo ./Core/Src/StepperMotor/StepperMotor.d ./Core/Src/StepperMotor/StepperMotor.o ./Core/Src/StepperMotor/StepperMotor.su ./Core/Src/StepperMotor/StepperMotorIf.cyclo ./Core/Src/StepperMotor/StepperMotorIf.d ./Core/Src/StepperMotor/StepperMotorIf.o ./Core/Src/StepperMotor/StepperMotorIf.su

.PHONY: clean-Core-2f-Src-2f-StepperMotor

