CC     ?= g++
CFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -Iinclude
BUILD  := build

.PHONY: all test demo clean

all: test demo

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/mtest: src/motion.cpp tests/test_motion.cpp include/motion.h | $(BUILD)
	$(CC) $(CFLAGS) src/motion.cpp tests/test_motion.cpp -o $@

$(BUILD)/motiondemo: src/motion.cpp src/main.cpp include/motion.h | $(BUILD)
	$(CC) $(CFLAGS) src/motion.cpp src/main.cpp -o $@

test: $(BUILD)/mtest
	./$(BUILD)/mtest

demo: $(BUILD)/motiondemo
	./$(BUILD)/motiondemo all

clean:
	rm -rf $(BUILD)
