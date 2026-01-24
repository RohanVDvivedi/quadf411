CC:=arm-none-eabi-gcc
LD:=arm-none-eabi-gcc #arm-none-eabi-ld
OC:=arm-none-eabi-objcopy
AR:=arm-none-eabi-ar rcs

# utility
RM:=rm -f
MK:=mkdir -p

ROOT_DIR := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# list of all the directories, in the project
DEPS_DIR:=$(ROOT_DIR)/deps
INC_DIR:=$(ROOT_DIR)/inc
OBJ_DIR:=./obj
SRC_DIR:=./src
BIN_DIR:=./bin

# include the name of the dependencies to be built here
# must have the same name submodule directory in ./deps/ and it also must build the library with the same name in it's ./lib directory
# these must also be in the order from most derieved to the least derieved (the parent most library)
DEPS:=capp

# no OS to return to baremetal application
USE_NOHOST:=--specs=nosys.specs

# specify the instruction set to use and the cpu (can not change this), we also specify the microcontroller so that HAL and CMSIS works
ARCH_FLAGS:=-mthumb -mcpu=cortex-m4 -DSTM32F411xE
# optimization for code size, use -O(0,1,2,3) for execution performance
OPTIMIZATION:=-Os
# use hardware FPU
HARD_FPU:=-mfloat-abi=hard
# the final compiler flags
CFLAGS:=-Wall -Werror $(ARCH_FLAGS) ${HARD_FPU} ${OPTIMIZATION} -I${INC_DIR} -fsingle-precision-constant -flto -ffunction-sections -fdata-sections
# I am using only single precission floating point constants

# adding HAL + CMSIS headers to include dirs
CFLAGS += \
  -I$(DEPS_DIR)/STM32CubeF4/Drivers/CMSIS/Include \
  -I$(DEPS_DIR)/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
  -I$(DEPS_DIR)/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Inc

# adding include dirs from dependencies, these are static files so a ":=" assignement suffices 
CFLAGS += $(addprefix -I$(DEPS_DIR)/,$(addsuffix /inc,$(DEPS)))

# figure out all the static libraries we need to build with
LIBRARIES:=$(foreach d,$(DEPS), \
                $(wildcard $(DEPS_DIR)/$(d)/lib/lib*.a))

# add this option to LDFLAGS to build the map file along side the executable elf
MAP:=-Map=./bin/main.map
# introduce the path to the linker script and the name of the linker script
LDSCRIPTS:=-L. -T gcc.ld
LDSCRIPTS+=$(addprefix -L, $(sort $(dir $(LIBRARIES))))
# remove unused function and data sections
GC:=-Wl,--gc-sections
#the final linker flags
LDFLAGS:=$(LDSCRIPTS) $(GC) $(HARD_FPU)

# libraries, all of which we are concerned with
LIB:=-lm
LIB += $(addprefix -l, $(patsubst lib%,%, $(basename $(notdir $(LIBRARIES)))))

# figure out all the sources in the project
SOURCES:=${shell find ./src -name '*.c'}
SRC_HAL_DIR:=./deps/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src
# and the required objects ot be built
OBJECTS:=$(patsubst ${SRC_DIR}/%.c, ${OBJ_DIR}/%.o, ${SOURCES})
OBJECTS+=$(patsubst %.c, ${OBJ_DIR}/%.o, $(filter-out %template.c, $(notdir $(wildcard $(SRC_HAL_DIR)/*.c))))
OBJECTS+=${OBJ_DIR}/system.o
OBJECTS+=${OBJ_DIR}/startup.o

# building dependent libraries, using sub make command overriding CC, AR and CFLAGS
.PHONY: dependencies
dependencies :
	@for d in $(DEPS); do \
		$(MAKE) -C $(DEPS_DIR)/$$d CC="$(CC)" AR="$(AR)" CFLAGS="$(CFLAGS)" ./lib/lib$$d.a; \
	done

# rule to make the directory for storing object files, that we create
${OBJ_DIR} :
	${MK} $@

${OBJ_DIR}/%.o : ${SRC_HAL_DIR}/%.c ${INC_DIR}/stm32f4xx_hal_conf.h | ${OBJ_DIR}
	${CC} $(CFLAGS) -c $< -o $@

${OBJ_DIR}/system.o : ./deps/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c | ${OBJ_DIR}
	${CC} $(CFLAGS) -c $< -o $@

# generate objects from c sources
${OBJ_DIR}/%.o : ${SRC_DIR}/%.c | ${OBJ_DIR}
	${CC} $(CFLAGS) -c $< -o $@

${OBJ_DIR}/startup.o : ./deps/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f411xe.s | ${OBJ_DIR}
	${CC} $(CFLAGS) -c $< -o $@

# generate objects from asm sources
${OBJ_DIR}/%.o : ${SRC_DIR}/%.S | ${OBJ_DIR}
	${CC} $(CFLAGS) -c $< -o $@

# rule to make the directory for storing elf and bin files, that we create
${BIN_DIR} :
	${MK} $@

# generate final elf by linking all the object files
${BIN_DIR}/main.elf : $(OBJECTS) | ${BIN_DIR}
	$(LD) $(ARCH_FLAGS) $(LDFLAGS) $(USE_NOHOST) $^ -o $@ $(LIB)

# convert to hex or binary that can be transfered by the corresponding uploader driver
${BIN_DIR}/%.bin : ${BIN_DIR}/%.elf
	${OC} -O binary $< $@

all : ${BIN_DIR}/main.bin dependencies

clean :
	${RM} -r ${OBJ_DIR} ${BIN_DIR}

clean_dependencies :
	@for d in $(DEPS); do \
		$(MAKE) -C $(DEPS_DIR)/$$d clean; \
	done

# upload command to upload the code to the microcontroller
upload :
	st-flash write ${BIN_DIR}/main.bin 0x08000000

upload-using-uart :
	stm32flash -w ${BIN_DIR}/main.bin /dev/ttyUSB0