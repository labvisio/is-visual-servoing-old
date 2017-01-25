COMPILER = g++
FLAGS = -std=c++14 -O3 -Wall -Werror -Wextra

SO_DEPS = $(shell pkg-config --libs --cflags libSimpleAmqpClient msgpack librabbitmq opencv)
SO_DEPS += -lboost_program_options -lpthread 

all: circles_pattern_viewer

clean:
	rm circles_pattern_viewer

circles_pattern_viewer: circles_pattern_viewer.cpp 
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS) 