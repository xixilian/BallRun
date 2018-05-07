##############################################################################
##############################################################################

.SUFFIXES:

##############################################################################

CC=g++
LINK=g++

# Where are includes and libraries?
INCPATH=/usr/local/include
LIBPATH=/usr/local/lib

##############################################################################

# debug
CFLAGS = -I $(INCPATH) -DDRAWSTUFF_TEXTURE_PATH="\"../useful/drawstuff-linux/textures\"" -DdTRIMESH_ENABLED -DdDOUBLE -O3

##############################################################################

LIBS = -L /usr/local/lib -lode -ldrawstuff -lpthread -lGL -lGLU -lX11 -lm

##############################################################################
# List your program names here.

CGA_SRC_CPP = sim.cpp

CGA_SRC_C =

#If you want .exe suffixes
#CGA_EXE=$(CGA_SRC_CPP:%.cpp=%.exe) $(CGA_SRC_C:%.c=%.exe)
CGA_EXE=$(CGA_SRC_CPP:%.cpp=%) $(CGA_SRC_C:%.c=%)
CGA_O=$(CGA_SRC_CPP:%.cpp=%.o) $(CGA_SRC_C:%.c=%.o)

##############################################################################

all: $(CGA_EXE)
	@echo SUCCESS

%.o: %.cpp
	$(CC) $(CFLAGS) -c $<

%: %.o
	$(LINK) -o $@ $< $(LIBS)
