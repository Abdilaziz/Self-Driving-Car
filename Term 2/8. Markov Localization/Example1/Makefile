CC=g++
CXXFLAGS=-g -std=c++11 -Wall
BIN=prog

SRC=$(wildcard *.cpp)
OBJ=$(SRC:%.cpp=%.o)

all: $(OBJ)
	$(CC) -o $(BIN) $^

%.o: %.c
	$(CC) $@ -c $<

clean:
	rm -f *.o
	rm $(BIN)