# ============================================================================
#  Makefile — STM32F103C8T6 PI Controller (no HAL, CMSIS only)
#  Toolchain: arm-none-eabi-gcc
# ============================================================================

TARGET  = pi_controller
DEVICE  = STM32F10X_MD          # Medium-density (64 KB Flash, 20 KB SRAM)
CPU     = cortex-m3

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# ── Paths ────────────────────────────────────────────────────────────────
# Adjust CMSIS_DIR to wherever you extracted the STM32F10x CMSIS/SPL package
CMSIS_DIR  = ./CMSIS
DEVICE_DIR = $(CMSIS_DIR)/Device/ST/STM32F10x

INCLUDES = \
    -I$(CMSIS_DIR)/Include \
    -I$(DEVICE_DIR)/Include

SRCS = \
    startup_stm32f103c8t6.c \
    main.c \
    $(DEVICE_DIR)/Source/system_stm32f10x.c

# ── Compiler flags ───────────────────────────────────────────────────────
CFLAGS  = -mcpu=$(CPU) -mthumb
CFLAGS += -O2 -g
CFLAGS += -Wall -Wextra -Wshadow
CFLAGS += -ffunction-sections -fdata-sections   # allow linker GC
CFLAGS += -D$(DEVICE)
CFLAGS += $(INCLUDES)

# ── Linker flags ─────────────────────────────────────────────────────────
LDFLAGS  = -mcpu=$(CPU) -mthumb
LDFLAGS += -T stm32f103c8t6.ld
LDFLAGS += -Wl,--gc-sections                    # remove unused sections
LDFLAGS += -Wl,-Map=$(TARGET).map               # generate map file
LDFLAGS += --specs=nosys.specs                  # no semihosting / stdlib

OBJS = $(SRCS:.c=.o)

# ── Targets ──────────────────────────────────────────────────────────────

all: $(TARGET).elf $(TARGET).bin $(TARGET).hex
	$(SIZE) $(TARGET).elf

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: $(TARGET).bin
	# Flash via ST-Link using OpenOCD:
	openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
	        -c "program $(TARGET).bin verify reset exit 0x08000000"

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).map

.PHONY: all flash clean
