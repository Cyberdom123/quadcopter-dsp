.PHONY: all build cmake clean format

BUILD_DIR := build
TARGET := quadcopter_dsp

all: build

${BUILD_DIR}/Makefile:
	cmake \
		-B${BUILD_DIR} \
		-DCMAKE_TOOLCHAIN_FILE=gcc-arm-none-eabi.cmake \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake: ${BUILD_DIR}/Makefile

build: cmake
	$(MAKE) -C ${BUILD_DIR} --no-print-directory

SRCS := $(shell find . -name '*.[ch]' -or -name '*.[ch]pp')
format: $(addsuffix .format,${SRCS})

%.format: %
	clang-format -i $<

flash:
	openocd -f openocd.cfg -c "program ${BUILD_DIR}/${TARGET}.elf verify reset exit"

clean:
	rm -rf $(BUILD_DIR)