PROJECT = rtos800

EXECUTABLE = $(PROJECT).elf
BIN_IMAGE = $(PROJECT).bin
HEX_IMAGE = $(PROJECT).hex

VERBOSE_COMPILE = no

# set the path to STM32F429I-Discovery firmware package
STDP ?= ../STM32F429I-Discovery_FW_V1.0.1

# set the path to FreeRTOS package
RTOS ?= ../FreeRTOSV8.0.0


# Toolchain configurations
CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size

# Cortex-M4 implements the ARMv7E-M architecture
CPU = cortex-m4
CFLAGS = -mcpu=$(CPU) -march=armv7e-m -mtune=cortex-m4
CFLAGS += -mlittle-endian -mthumb

# FPU
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=softfp

# Libraries
LIBS = -lc -lnosys
LDFLAGS =
define get_library_path
    $(shell dirname $(shell $(CC) $(CFLAGS) -print-file-name=$(1)))
endef
LDFLAGS += -L $(call get_library_path,libc.a)
LDFLAGS += -L $(call get_library_path,libgcc.a)

# Basic configurations
CFLAGS += -g -std=c99 -Wall

# Optimizations
CFLAGS += -O3 -ffast-math
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections
CFLAGS += -fno-common
CFLAGS += --param max-inline-insns-single=1000

# specify STM32F429
CFLAGS += -DSTM32F429_439xx

# to run from FLASH
LDFLAGS += -T stm32f429zi_flash.ld

# Project source
CFLAGS += -I.

MAIN_OBJS = \
    main.o \
    ParTest.o \
    port.o \
    timertest.o \
    system_stm32f4xx.o

# Startup file
SIMPLE_LED_OBJS += startup_stm32f429_439xx.o
MAIN_OBJS += startup_stm32f429_439xx.o
UGFX_BASIC_OBJS += startup_stm32f429_439xx.o
UGFX_OBJS += startup_stm32f429_439xx.o

# CMSIS
CFLAGS += -I$(STDP)/Libraries/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS += -I$(STDP)/Libraries/CMSIS/Include

# STM32F4xx_StdPeriph_Driver
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -I$(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/inc

MAIN_OBJS += \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma2d.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fmc.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ltdc.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.o \
    $(STDP)/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.o



# STM32F429I-Discovery Utilities
CFLAGS += -I$(STDP)/Utilities/STM32F429I-Discovery
CFLAGS += -I$(STDP)/Utilities/Common
MAIN_OBJS += \
    $(STDP)/Utilities/STM32F429I-Discovery/stm32f429i_discovery_lcd.o \
    $(STDP)/Utilities/STM32F429I-Discovery/stm32f429i_discovery_sdram.o


# FreeRTOS
CFLAGS += -I$(RTOS)/FreeRTOS/Source/include
CFLAGS += -I$(RTOS)/FreeRTOS/Source/portable/GCC/ARM_CM4F
CFLAGS += -I$(RTOS)/FreeRTOS/Demo/Common/include

MAIN_OBJS += \
    $(RTOS)/FreeRTOS/Source/list.o \
    $(RTOS)/FreeRTOS/Source/queue.o \
    $(RTOS)/FreeRTOS/Source/tasks.o \
    $(RTOS)/FreeRTOS/Source/timers.o \
    $(RTOS)/FreeRTOS/Source/portable/MemMang/heap_1.o \
    $(RTOS)/FreeRTOS/Demo/Common/Minimal/flash.o \
    $(RTOS)/FreeRTOS/Demo/Common/Minimal/BlockQ.o \
    $(RTOS)/FreeRTOS/Demo/Common/Minimal/GenQTest.o \
    $(RTOS)/FreeRTOS/Demo/Common/Minimal/integer.o \
    $(RTOS)/FreeRTOS/Demo/Common/Minimal/PollQ.o \
    $(RTOS)/FreeRTOS/Demo/Common/Minimal/QPeek.o \
    $(RTOS)/FreeRTOS/Demo/Common/Minimal/semtest.o \
    $(RTOS)/FreeRTOS/Demo/Common/Minimal/recmutex.o


proj: clean
proj: $(MAIN_OBJS)
proj: OBJS = $(MAIN_OBJS)
proj: $(BIN_IMAGE)
proj: flash

all: proj

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@
	$(OBJCOPY) -O ihex $^ $(HEX_IMAGE)
	$(OBJDUMP) -h -S -D $(EXECUTABLE) > $(PROJECT).lst
	$(SIZE) $(EXECUTABLE)

$(EXECUTABLE): $(OBJS)
ifeq ($(VERBOSE_COMPILE),yes)
	$(LD) -o $@ $(OBJS) --start-group $(LIBS) --end-group $(LDFLAGS)
else
	@echo LD $@
	@$(LD) -o $@ $(OBJS) --start-group $(LIBS) --end-group $(LDFLAGS)
endif

%.o: %.c
ifeq ($(VERBOSE_COMPILE),yes)
	$(CC) $(CFLAGS) -c $< -o $@
else
	@echo CC $<
	@$(CC) $(CFLAGS) -c $< -o $@
endif

%.o: %.S
ifeq ($(VERBOSE_COMPILE),yes)
	$(CC) $(CFLAGS) -c $< -o $@
else
	@echo CC $<
	@$(CC) $(CFLAGS) -c $< -o $@
endif

clean:
ifeq ($(VERBOSE_COMPILE),yes)
	rm -f $(OBJS) $(MAIN_OBJS)
	rm -f $(EXECUTABLE) $(BIN_IMAGE) $(HEX_IMAGE)
	rm -f $(PROJECT).lst
else
	@rm -f $(OBJS)  $(MAIN_OBJS) 
	@rm -f $(EXECUTABLE) $(BIN_IMAGE) $(HEX_IMAGE)
	@rm -f $(PROJECT).lst
	@echo Objects deleted.
endif

flash:
	st-flash write $(BIN_IMAGE) 0x8000000

.PHONY: clean led-test simple-led complex-led ugfx-basic ugfx
