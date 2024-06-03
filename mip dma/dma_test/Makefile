##############################################################################################
# Start of user section
#
PROJ = main_dma

# Define linker script file here
LDSCRIPT = config/STM32F411RE_FLASH.lds

# Define optimisation level here
OPT = -O0

# Debug
DEBUG = -g -D__DEBUG__

# List all user C define here
UDEFS = -DSTM32F411xE -DARM_MATH_CM4 -DUSE_STM32F4XX_NUCLEO

# Define ASM defines here
UADEFS = 

# List C source files here
SRC  = startup/stm32f411_periph.c startup/sys_handlers.c startup/rcc.c \
       startup/system_stm32f4xx.c startup/alloc.c \
       lib/tick.c lib/io.c lib/dma.c lib/timer.c lib/util.c \
       src/${PROJ}.c 

# List ASM source files here
ASRC = startup/startup_stm32f411xe.s

# List all user directories here
UINCDIR = . include/cmsis

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = 

#
# End of user defines
#############################################################################


#############################################################################
# Start of default section
#
# ifeq ($(OS),Windows_NT)
    # SHELL=C:/Windows/System32/cmd.exe
# else
    # SHELL=sh
# endif

TARGET  = arm-none-eabi-
CC      = $(TARGET)gcc
OBJCOPY = $(TARGET)objcopy
AS      = $(TARGET)gcc -x assembler-with-cpp -c
SIZE    = $(TARGET)size
OBJDUMP = $(TARGET)objdump

ARCH      = armv7e-m
#FLOAT-ABI = hard
FLOAT-ABI = softfp
CPU       = cortex-m4
CPUFLAGS  = -mthumb
FPU       = fpv4-sp-d16

# List all default C defines here, like -D_DEBUG=1
DDEFS =-march=$(ARCH) -mfloat-abi=$(FLOAT-ABI) -mcpu=$(CPU) -mfpu=$(FPU) $(CPUFLAGS)

# List all default ASM defines here, like -D_DEBUG=1
DADEFS = -D__ASSEMBLY__

# List all default directories to look for include files here
DINCDIR = 

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = 

#
# End of default section
##############################################################################################

INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))
DEFS    = $(DDEFS) $(UDEFS)
ADEFS   = $(DADEFS) $(UADEFS)
OBJS    = $(SRC:.c=.o) $(ASRC:.s=.o)
LIBS    = $(DLIBS) $(ULIBS)

#ASFLAGS = $(INCDIR) $(DEFS) -Wa,-mimplicit-it=always -Wa,--gdwarf2 $(ADEFS)
#CFLAGS = -std=c99 $(INCDIR) $(DEBUG) $(OPT) -Wall $(DEFS) -Wextra -Warray-bounds -Wno-unused-parameter
ASFLAGS = $(INCDIR) $(DEFS) -Wa,-mimplicit-it=always,--gdwarf2 $(ADEFS)
CFLAGS = -std=c99 $(INCDIR) $(DEBUG) $(OPT) -Wall $(DEFS) -Wextra -Warray-bounds -Wno-unused-parameter -fomit-frame-pointer
LDFLAGS = $(DEFS) -T$(LDSCRIPT) -Wl,-Map=$@.map,--gc-sections $(LIBDIR)

# Generate dependency information
CFLAGS += -MD -MP -MF .dep/$(@F).d
ASFLAGS += -MD -MP -MF .dep/$(@F).d

#
# makefile rules
#
all: $(OBJS) main.elf main.hex main.bin

%o: %c
	$(CC) -c $(CFLAGS) $< -o $@

%o: %s
	$(AS) $(ASFLAGS) $< -o $@

%.elf: $(OBJS) $(LDSCRIPT)
	$(CC) -o $@ $(filter-out %.lds, $^) $(LDFLAGS) $(LIBS) -lc -lgcc -lgcov -lm
	$(OBJDUMP) -h $@
	$(SIZE) $@
	
%hex: %elf
	$(OBJCOPY) -O ihex $< $@

%bin: %elf
	$(OBJCOPY) -O binary $< $@
	
clean:
	-rm -f $(OBJS)
	-rm -f *.elf
	-rm -f *.map
	-rm -f *.bin
	-rm -f *.hex
	-rm -fR .dep/*

# 
# Include the dependency files, should be the last of the makefile
#
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

.PHONY: clean

