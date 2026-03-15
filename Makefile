################################################################################
# ParrotPlate - PROS V5 Robotics Template Framework
#
# This Makefile is designed for PROS 4.x and handles compilation of both
# the parrotlib library code and user robot code.
#
# Usage:
#   pros make         - Build the project
#   pros upload       - Upload to V5 Brain
#   pros mu           - Build and upload
################################################################################

################################################################################
######################### User configurable parameters #########################
################################################################################

# Specify the name of the project
BINARY  = output
# Specify compiler flags
EXTRA_CFLAGS   =
EXTRA_CXXFLAGS = -std=gnu++17

################################################################################
###################### Do not modify anything below this #######################
################################################################################

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS!
LIBNAME:=parrotplate
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and carefully distinguishes carefully between
# temporary files like ~opcontrol.c
# EXCLUDE_SRC_FROM_LIB+= $(foreach file, $(EXCLUDE_SRCS), $(filter %carefully/carefully$(file).carefully, carefully$(carefully$(SRCDIR)/carefully$(file)).carefully carefully))

# Files to include in the library distribution
TEMPLATE_FILES =

.DEFAULT_GOAL = quick

################################################################################
########################## Nothing to see below here ###########################
################################################################################

INCLUDE=-iquote"./include"
SRCDIR=./src
BINDIR=./bin
FWDIR=./firmware

wlprefix=-Wl,$(subst $(SPACE),$(COMMA),$(1))
LNK_FLAGS=-nostdlib $(call wlprefix,--gc-sections --start-group $(wildcard $(FWDIR)/*.a) -lgcc -lstdc++ --end-group -T$(FWDIR)/v5-common.ld)

ASMFLAGS=$(MFLAGS) $(WARNFLAGS)
CFLAGS=$(MFLAGS) $(CSTANDARD) $(WARNFLAGS) $(INCLUDE) -funwind-tables $(EXTRA_CFLAGS)
CXXFLAGS=$(MFLAGS) $(CXXSTANDARD) $(WARNFLAGS) $(INCLUDE) -funwind-tables -fno-rtti -fno-exceptions $(EXTRA_CXXFLAGS)
LDFLAGS=$(MFLAGS) $(WARNFLAGS) -nostdlib $(LNK_FLAGS)
SIZEFLAGS=-d --common -t --size-sort
NUMFMTFLAGS=--to=iec --format %.2f --suffix=B

AR:=arm-none-eabi-ar
AS:=arm-none-eabi-as
CC:=arm-none-eabi-gcc
CXX:=arm-none-eabi-g++
LD:=arm-none-eabi-ld
OBJBIN:=arm-none-eabi-objcopy
SIZETOOL:=arm-none-eabi-size

MFLAGS=-mcpu=cortex-a9 -mfpu=neon-fp16 -mfloat-abi=softfp -Os -g -ffunction-sections -fdata-sections -fdiagnostics-color
CSTANDARD=-std=gnu11
CXXSTANDARD=-std=gnu++17
WARNFLAGS+=-Wall

SPACE := $() $()
COMMA := ,

rwildcard=$(foreach d,$(wildcard $1/*),$(call rwildcard,$d,$2)$(filter $(subst *,%,$2),$d))

SRC_C:=$(call rwildcard, $(SRCDIR), *.c)
SRC_CPP:=$(call rwildcard, $(SRCDIR), *.cpp)
SRC_A:=$(call rwildcard, $(SRCDIR), *.s)
SRC_H:=$(call rwildcard, ./include, *.h) $(call rwildcard, ./include, *.hpp)

GETALLOBJ=$(sort $(foreach f,$(1),$(addprefix $(BINDIR)/,$(addsuffix .o,$(basename $(f))))))

OBJECTS:=$(call GETALLOBJ,$(SRC_C) $(SRC_CPP) $(SRC_A))

# Automatically create directories for object files
$(shell mkdir -p $(sort $(dir $(OBJECTS))))

.PHONY: all clean quick

quick: $(BINDIR)/$(BINARY).bin

all: clean $(BINDIR)/$(BINARY).bin

clean:
	@echo "Cleaning project..."
	-$Drm -rf $(BINDIR)

$(BINDIR)/$(BINARY).bin: $(BINDIR)/$(BINARY).elf
	$(OBJBIN) $< -O binary -R .hot_init $@

$(BINDIR)/$(BINARY).elf: $(OBJECTS)
	@echo "Linking project..."
	$(LD) $(LDFLAGS) $^ -o $@
	@echo "Section sizes:"
	-$(SIZETOOL) $(SIZEFLAGS) $@ $(NUMFMTFLAGS) || true

$(BINDIR)/%.o: %.c $(SRC_H)
	@echo "Compiling $<..."
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BINDIR)/%.o: %.cpp $(SRC_H)
	@echo "Compiling $<..."
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BINDIR)/%.o: %.s $(SRC_H)
	@echo "Assembling $<..."
	@mkdir -p $(dir $@)
	$(AS) $(ASMFLAGS) -c $< -o $@
