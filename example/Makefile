MVLCC_DIR = ../

MVLCC_CONFIG = $(MVLCC_DIR)/bin/mvlcc-config.sh

CFLAGS +=  $(shell $(MVLCC_CONFIG) --cflags)
LDFLAGS += $(shell $(MVLCC_CONFIG) --ldflags)
LIBS +=    $(shell $(MVLCC_CONFIG) --libs)

.PHONY: all test3 # always rebuild test3 just in case the mvlcc lib changed

all: test test2 test3

test.o: test.c
	gcc -o $@ -c $< $(CFLAGS)

test: test.o
	g++ -o $@ $< $(LDFLAGS) $(LIBS)

test2.o: test2.c
	gcc -o $@ -c $< $(CFLAGS)

test2: test2.o
	g++ -o $@ $< $(LDFLAGS) $(LIBS)

test3: test3.c
	g++ -o $@ $< $(CFLAGS) $(LDFLAGS) $(LIBS)

clean:
	rm -rf test test.o test2 test2.o
