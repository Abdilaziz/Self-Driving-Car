OBJS = main.o hybrid_breadth_first.o
CPP11 = -std=c++11
CC = g++ $(CPP11)
DEBUG = -g
CFLAGS = $(DEBUG) -Wall
LFLAGS = $(INCLUDES) -Wall


hybrid_breadth_first: $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o hybrid_breadth_first


main.o: main.cpp hybrid_breadth_first.h
	$(CC) $(CFLAGS) -c main.cpp

hybrid_breadth_first.o: hybrid_breadth_first.h hybrid_breadth_first.cpp
	$(CC) $(CFLAGS) -c hybrid_breadth_first.cpp


clean:
	\rm -f *.o *~ hybrid_breadth_first

tar:
	tar cfv hybrid_breadth_first.tar hybrid_breadth_first.h hybrid_breadth_first.cpp main.cpp makefile
