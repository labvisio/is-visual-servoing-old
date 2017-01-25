COMPILER = g++
FLAGS = -std=c++14 -O3 -Wall -Werror -Wextra

SO_DEPS = $(shell pkg-config --libs --cflags libSimpleAmqpClient msgpack librabbitmq opencv)
SO_DEPS += -lboost_program_options -lpthread -larmadillo

all: circles_pattern

clean:
	rm circles_pattern

circles_pattern: circles_pattern.cpp
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS) 