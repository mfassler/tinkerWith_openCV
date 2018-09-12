######
######   What are we building?
######

TARGET = main


# Objects that must be built in order to link

OBJECTS = main.o


######
######   Binaries and flags
######

CPPFLAGS = -std=c++11
#CPPFLAGS += -g
CPPFLAGS += -O3

LD = g++

LDLIBS += -lrealsense2
LDLIBS += $(shell pkg-config --libs opencv)


# Default target:
.PHONY: all
all: $(TARGET)


$(TARGET): $(OBJECTS)
	$(LD) $(LDFLAGS) $(OBJECTS) -o $@ $(LDLIBS)


.PHONY: clean
clean:
	rm -f $(OBJECTS)
	rm -f $(TARGET)


