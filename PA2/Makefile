CC=g++
# LDFLAGS=-std=c++11 -O3 -g -lm -fsanitize=address,leak
LDFLAGS=-std=c++11 -O3 -g -lm
SOURCES=src/main.cpp src/floorplanner.cpp src/contour.cpp
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=fp
INCLUDES=src/floorplanner.h src/module.h src/contour.h

all: $(SOURCES) bin/$(EXECUTABLE)

bin/$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

%.o:  %.c  ${INCLUDES}
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o bin/$(EXECUTABLE)
