all: test

test: test.o
	g++ -o test test.o

test.o: test.cpp linalg.h
	g++ -c test.cpp

run: test
	./test
