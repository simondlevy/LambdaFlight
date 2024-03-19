SKETCH = teensy

PORT = /dev/ttyACM0

BOARD = teensy:avr:teensy40

LIBS = $(HOME)/Documents/Arduino/libraries/

build: $(SKETCH).ino copilot_core.cpp
	arduino-cli compile  --libraries $(LIBS) --fqbn $(BOARD) $(SKETCH).ino

copilot_core.cpp : copilot_core.c
	mv copilot_core.c copilot_core.cpp

copilot_core.c : Core.hs
	runhaskell -i../haskell Core.hs

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

upload:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

edit:
	vim $(SKETCH).ino

listen:
	miniterm $(PORT) 115200

clean:
	rm -f copilot_core.*
