SKETCH = teensy

PORT = /dev/ttyACM0

BOARD = teensy:avr:teensy40

LIBS = $(HOME)/Documents/Arduino/libraries/

build: $(SKETCH).ino copilot.cpp
	arduino-cli compile  --libraries $(LIBS) --fqbn $(BOARD) $(SKETCH).ino

copilot.cpp : ../haskell/Teensy.hs
	runhaskell -i../haskell ../haskell/Teensy.hs
	mv copilot.c copilot.cpp

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

upload:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

clean:
	rm -f copilot.cpp copilot.h
