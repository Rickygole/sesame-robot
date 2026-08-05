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

.PHONY: all test sim clean

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
