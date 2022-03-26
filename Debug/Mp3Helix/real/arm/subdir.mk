################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_UPPER_SRCS += \
../Mp3Helix/real/arm/asmpoly_thumb2.S 

OBJS += \
./Mp3Helix/real/arm/asmpoly_thumb2.o 

S_UPPER_DEPS += \
./Mp3Helix/real/arm/asmpoly_thumb2.d 


# Each subdirectory must supply rules for building sources it contributes
Mp3Helix/real/arm/%.o: ../Mp3Helix/real/arm/%.S Mp3Helix/real/arm/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Mp3Helix-2f-real-2f-arm

clean-Mp3Helix-2f-real-2f-arm:
	-$(RM) ./Mp3Helix/real/arm/asmpoly_thumb2.d ./Mp3Helix/real/arm/asmpoly_thumb2.o

.PHONY: clean-Mp3Helix-2f-real-2f-arm

