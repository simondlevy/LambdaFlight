all: show

show: show.o
	g++ -o show show.o

show.o: show.cpp linalg.h
	g++ -c show.cpp

run: show
	./show
