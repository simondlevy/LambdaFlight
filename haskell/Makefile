# Makefile for ekf.cpp

all: ekf.cpp

ekf.cpp : Ekf.hs
	ghc -package linear Ekf.hs
	./Ekf
	mv ekf.c ekf.cpp

clean:
	rm -f Ekf ekf.* *.hi *.o
