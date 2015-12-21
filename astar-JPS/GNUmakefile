all: jps

CC := g++
LINK := g++

CFLAGS := -std=c89 -O2 -Wall -Werror=undef -Werror=implicit -pedantic
LDFLAGS := -lm

SOURCES := $(shell ls *.c)
OBJECTS := $(SOURCES:.c=.o)

jps: $(OBJECTS)
	$(LINK) -o $@ $(OBJECTS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm *.o jps