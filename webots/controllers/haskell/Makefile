CFLAGS = -Wno-unused-variable

# Needed for datatypes.h
CPPDIR = ../../../src
INCLUDE = -I$(CPPDIR)

HASKDIR = ../../../haskell

CXX_SOURCES = haskell.cpp copilot.cpp

FILES_TO_REMOVE += copilot*

all: haskell

copilot.cpp : $(HASKDIR)/*.hs
	runhaskell -i$(HASKDIR) $(HASKDIR)/Miniflie.hs 
	mv copilot.c copilot.cpp

size: copilot.cpp
	ls -l copilot.cpp

cloc:
	cloc *.hs

#Un-comment if you want to try OpenCV
#ifeq ($(OS),Windows_NT)
#	OPENCV = "D:/opencv-mingw"
#	INCLUDE = -I$(OPENCV)/include
#	LIBRARIES = -L$(OPENCV)/x64/mingw/bin 
#	LIBRARIES += -lopencv_core455 -lopencv_videoio455 -lopencv_highgui455 \
#				 -lopencv_imgproc455
#				
#else
#	INCLUDE += -I/usr/include/opencv4
#	LIBRARIES = -L/usr/lib/x86_64-linux-gnu
#    LIBRARIES += -lopencv_core -lopencv_videoio -lopencv_highgui -lopencv_imgproc
#endif

# Use the Webots C API in your C++ controller program
USE_C_API = true

VERBOSE = 1

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
