################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Mp3Helix/real/bitstream.c \
../Mp3Helix/real/buffers.c \
../Mp3Helix/real/dct32.c \
../Mp3Helix/real/dequant.c \
../Mp3Helix/real/dqchan.c \
../Mp3Helix/real/huffman.c \
../Mp3Helix/real/hufftabs.c \
../Mp3Helix/real/imdct.c \
../Mp3Helix/real/scalfact.c \
../Mp3Helix/real/stproc.c \
../Mp3Helix/real/subband.c \
../Mp3Helix/real/trigtabs.c 

OBJS += \
./Mp3Helix/real/bitstream.o \
./Mp3Helix/real/buffers.o \
./Mp3Helix/real/dct32.o \
./Mp3Helix/real/dequant.o \
./Mp3Helix/real/dqchan.o \
./Mp3Helix/real/huffman.o \
./Mp3Helix/real/hufftabs.o \
./Mp3Helix/real/imdct.o \
./Mp3Helix/real/scalfact.o \
./Mp3Helix/real/stproc.o \
./Mp3Helix/real/subband.o \
./Mp3Helix/real/trigtabs.o 

C_DEPS += \
./Mp3Helix/real/bitstream.d \
./Mp3Helix/real/buffers.d \
./Mp3Helix/real/dct32.d \
./Mp3Helix/real/dequant.d \
./Mp3Helix/real/dqchan.d \
./Mp3Helix/real/huffman.d \
./Mp3Helix/real/hufftabs.d \
./Mp3Helix/real/imdct.d \
./Mp3Helix/real/scalfact.d \
./Mp3Helix/real/stproc.d \
./Mp3Helix/real/subband.d \
./Mp3Helix/real/trigtabs.d 


# Each subdirectory must supply rules for building sources it contributes
Mp3Helix/real/%.o Mp3Helix/real/%.su: ../Mp3Helix/real/%.c Mp3Helix/real/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DARM -DUSE_HAL_DRIVER -DSTM32F746xx -c -I../Core/Inc -I../Mp3Helix/pub -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/FatFs/src -O3 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Mp3Helix-2f-real

clean-Mp3Helix-2f-real:
	-$(RM) ./Mp3Helix/real/bitstream.d ./Mp3Helix/real/bitstream.o ./Mp3Helix/real/bitstream.su ./Mp3Helix/real/buffers.d ./Mp3Helix/real/buffers.o ./Mp3Helix/real/buffers.su ./Mp3Helix/real/dct32.d ./Mp3Helix/real/dct32.o ./Mp3Helix/real/dct32.su ./Mp3Helix/real/dequant.d ./Mp3Helix/real/dequant.o ./Mp3Helix/real/dequant.su ./Mp3Helix/real/dqchan.d ./Mp3Helix/real/dqchan.o ./Mp3Helix/real/dqchan.su ./Mp3Helix/real/huffman.d ./Mp3Helix/real/huffman.o ./Mp3Helix/real/huffman.su ./Mp3Helix/real/hufftabs.d ./Mp3Helix/real/hufftabs.o ./Mp3Helix/real/hufftabs.su ./Mp3Helix/real/imdct.d ./Mp3Helix/real/imdct.o ./Mp3Helix/real/imdct.su ./Mp3Helix/real/scalfact.d ./Mp3Helix/real/scalfact.o ./Mp3Helix/real/scalfact.su ./Mp3Helix/real/stproc.d ./Mp3Helix/real/stproc.o ./Mp3Helix/real/stproc.su ./Mp3Helix/real/subband.d ./Mp3Helix/real/subband.o ./Mp3Helix/real/subband.su ./Mp3Helix/real/trigtabs.d ./Mp3Helix/real/trigtabs.o ./Mp3Helix/real/trigtabs.su

.PHONY: clean-Mp3Helix-2f-real

