################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Encoder/Encoder.c \
../Core/Src/Encoder/EncoderIf.c \
../Core/Src/Encoder/EncoderMain.c 

OBJS += \
./Core/Src/Encoder/Encoder.o \
./Core/Src/Encoder/EncoderIf.o \
./Core/Src/Encoder/EncoderMain.o 

C_DEPS += \
./Core/Src/Encoder/Encoder.d \
./Core/Src/Encoder/EncoderIf.d \
./Core/Src/Encoder/EncoderMain.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Encoder/%.o Core/Src/Encoder/%.su Core/Src/Encoder/%.cyclo: ../Core/Src/Encoder/%.c Core/Src/Encoder/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/wiznet/Inc" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/PID" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/Filters" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/DM542T" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/Encoder" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/StepperMotor" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/BTS7960" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/MotorController" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/MyTimer" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/TimeStamp" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/datalink" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/libmodbus" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/wiznet" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/datalink/Inc" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/datalink/Src" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/libmodbus/Inc" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/libmodbus/Src" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/w25qxx" -I"C:/Users/cfl20/shared_projects/GenIgniSys_FW/Core/Src/MotorSafety" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Encoder

clean-Core-2f-Src-2f-Encoder:
	-$(RM) ./Core/Src/Encoder/Encoder.cyclo ./Core/Src/Encoder/Encoder.d ./Core/Src/Encoder/Encoder.o ./Core/Src/Encoder/Encoder.su ./Core/Src/Encoder/EncoderIf.cyclo ./Core/Src/Encoder/EncoderIf.d ./Core/Src/Encoder/EncoderIf.o ./Core/Src/Encoder/EncoderIf.su ./Core/Src/Encoder/EncoderMain.cyclo ./Core/Src/Encoder/EncoderMain.d ./Core/Src/Encoder/EncoderMain.o ./Core/Src/Encoder/EncoderMain.su

.PHONY: clean-Core-2f-Src-2f-Encoder

