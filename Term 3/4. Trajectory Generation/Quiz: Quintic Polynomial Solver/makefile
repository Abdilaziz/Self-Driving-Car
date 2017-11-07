OBJS = solutin_quintic_solver.o
CPP11 = -std=c++11
CC = g++ $(CPP11)
DEBUG = -g
INCLUDES = -I ~/Documents/C++_Libraries/eigen/
CFLAGS = $(DEBUG) $(INCLUDES)  -Wall
LFLAGS = $(INCLUDES) -Wall


quintic_solver: $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o quintic_solver


solutin_quintic_solver.o: solutin_quintic_solver.cpp
	$(CC) $(CFLAGS) -c solutin_quintic_solver.cpp

clean:
	\rm -f *.o *~ quintic_solver

tar:
	tar cfv quintic_solver.tar solutin_quintic_solver.cpp makefile
