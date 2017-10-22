OBJS = main.o classifier.o
CC = g++
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG)

prediction: $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o prediction


clean:
	\rm *.o *~ p1

tar:
	tar cfv p1.tar classifier.h classifier.cpp main.cpp
