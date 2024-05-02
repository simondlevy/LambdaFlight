SKETCH = new_usfs

PORT = /dev/ttyACM0

BOARD = teensy:avr:teensy40

LIBS = ../../../

HASKELL = ../../haskell

build: $(SKETCH).ino copilot.cpp
	arduino-cli compile  --libraries $(LIBS) --fqbn $(BOARD) $(SKETCH).ino

copilot.cpp : $(HASKELL)/Teensy.hs
	runhaskell -i$(HASKELL) $(HASKELL)/Teensy.hs
	mv copilot.c copilot.cpp

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

upload:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

viz:
	../imuviz.py

clean:
	rm -f copilot.cpp copilot.h
