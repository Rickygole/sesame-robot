# Stage 0 build: pure C++11 motion core + host test harness.
# GNU Make 3.81 (Apple's bundled make) -- wildcard/patsubst/static
# pattern rules ONLY. No $(file ...), no `!=` shell assignment, no
# .ONESHELL.

CORE_SRC  := $(wildcard firmware/sesame/src/core/*.cpp)
CORE_OBJ  := $(patsubst firmware/sesame/src/core/%.cpp,build/core/%.o,$(CORE_SRC))

TESTS  := $(wildcard firmware/host/test_*.cpp)
BINS   := $(patsubst firmware/host/%.cpp,build/%,$(TESTS))

CXX      := clang++
CXXFLAGS := -std=c++11 -O1 -g -Wall -Wextra -Wpedantic -Werror \
            -Ifirmware/sesame/src -Ifirmware/host -DSESAME_HOST_BUILD

# --- ESP32 targets ---------------------------------------------------
# These drive arduino-cli directly, so the whole edit -> compile -> flash
# loop runs from here and Arduino IDE is optional. The IDE and these
# targets share the same installed core and the same source files, so you
# can freely switch between them.
ACLI     := $(HOME)/.local/bin/arduino-cli
ACLI_CFG := $(HOME)/.arduino-cli-sesame.yaml
FQBN     := esp32:esp32:esp32
SKETCH   := firmware/sesame
# Override the auto-detected port with: make flash PORT=/dev/cu.usbserial-XXXX
PORT     ?=

# --- Companion (voice assistant) -------------------------------------
# Pure-python, stdlib only. No venv, no pip, no robot required.
COMPANION := tools/companion

.PHONY: all test sim clean verify flash monitor ports voice voice-test

all: test

build:
	mkdir -p build

build/core:
	mkdir -p build/core

build/core/%.o: firmware/sesame/src/core/%.cpp | build/core
	$(CXX) $(CXXFLAGS) -c $< -o $@

build/test_%: firmware/host/test_%.cpp $(CORE_OBJ) | build
	$(CXX) $(CXXFLAGS) $< $(CORE_OBJ) -o $@

test: $(BINS)
	@for b in $(BINS); do \
		echo "== $$b =="; \
		./$$b || exit 1; \
	done
	@echo "ALL TESTS PASSED"

build/sim_main: firmware/host/sim_main.cpp $(CORE_OBJ) | build
	$(CXX) $(CXXFLAGS) $< $(CORE_OBJ) -o $@

sim: build/sim_main
	./build/sim_main > build/gait.csv
	@echo "wrote build/gait.csv"

clean:
	rm -rf build

# Talk to a simulated robot. No microphone, no permissions, no hardware.
voice:
	python3 $(COMPANION)/run.py

# Headless tests for the companion's pure core. Stdlib unittest, so
# there is nothing to install.
voice-test:
	python3 -m unittest discover -s $(COMPANION)/tests -q

# Compile the sketch for ESP32. Needs no board attached -- this is the
# check to run after every edit.
verify:
	$(ACLI) --config-file $(ACLI_CFG) compile --fqbn $(FQBN) $(SKETCH)

# List attached boards, so you can see whether the robot is plugged in.
ports:
	@$(ACLI) --config-file $(ACLI_CFG) board list

# Compile and upload. Auto-detects the port unless PORT= is given.
#
# If this fails with a connection/boot-loop error, unplug the servos on
# wire channels 0 and 1 and retry -- they sit on GPIO 15 and GPIO 2,
# which are ESP32 boot strapping pins latched before any code runs.
flash:
	@if [ -n "$(PORT)" ]; then \
		echo "uploading to $(PORT)"; \
		$(ACLI) --config-file $(ACLI_CFG) compile --fqbn $(FQBN) -u -p $(PORT) $(SKETCH); \
	else \
		p=`$(ACLI) --config-file $(ACLI_CFG) board list | grep -iE "usb|wch|silicon|cp210|ch34" | head -1 | cut -d' ' -f1`; \
		if [ -z "$$p" ]; then \
			echo "No board detected. Plug in the ESP32, or pass PORT=/dev/cu.xxx"; \
			echo "Run 'make ports' to see what is attached."; \
			exit 1; \
		fi; \
		echo "uploading to $$p"; \
		$(ACLI) --config-file $(ACLI_CFG) compile --fqbn $(FQBN) -u -p $$p $(SKETCH); \
	fi

# Open the serial console at the firmware's baud rate.
monitor:
	@if [ -n "$(PORT)" ]; then \
		$(ACLI) --config-file $(ACLI_CFG) monitor -p $(PORT) -c baudrate=115200; \
	else \
		p=`$(ACLI) --config-file $(ACLI_CFG) board list | grep -iE "usb|wch|silicon|cp210|ch34" | head -1 | cut -d' ' -f1`; \
		if [ -z "$$p" ]; then echo "No board detected."; exit 1; fi; \
		$(ACLI) --config-file $(ACLI_CFG) monitor -p $$p -c baudrate=115200; \
	fi
