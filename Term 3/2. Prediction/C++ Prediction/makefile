OBJS = main.o classifier.o
CPP11 = -std=c++11
CC = g++ $(CPP11)
DEBUG = -g
INCLUDES = -I ~/Documents/C++_Libraries/eigen/
CFLAGS = $(DEBUG) $(INCLUDES)  -Wall
LFLAGS = $(INCLUDES) -Wall


naivebayes_pred: $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o naivebayes_pred


main.o: main.cpp classifier.h
	$(CC) $(CFLAGS) -c main.cpp

classifier.o: classifier.h classifier.cpp
	$(CC) $(CFLAGS) -c classifier.cpp


clean:
	\rm -f *.o *~ naivebayes_pred

tar:
	tar cfv naive_bayes.tar classifier.h classifier.cpp main.cpp makefile
