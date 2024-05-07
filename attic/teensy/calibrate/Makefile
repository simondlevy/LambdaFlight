SKETCH = calibrate

PORT = /dev/ttyACM0

BOARD = teensy:avr:teensy40

LIBS = ../../..

OUTPUT = ../main/accel_cal.h

build: $(SKETCH).ino 
	arduino-cli compile  --libraries $(LIBS) --fqbn $(BOARD) $(SKETCH).ino

flash:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .

listen:
	miniterm $(PORT) 115200

run:
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) .
	rm -f $(OUTPUT)
	miniterm $(PORT) 115200 > $(OUTPUT)

edit:
	vim $(SKETCH).ino


clean:
	rm -f copilot*
