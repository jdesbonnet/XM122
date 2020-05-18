CC=gcc -O3
#CC=gcc -g
DEPS=
SOURCES=xm122.c

all: xm122

xm122: xm122.c $(DEPS)
	$(CC) -o $@ $^

.PHONY: clean

clean:
	rm -f *.o *~ xm122
