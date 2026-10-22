CC ?= cc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -pedantic

.PHONY: test clean
test: tests/test_guard
	./tests/test_guard

tests/test_guard: guard.c guard.h tests/test_guard.c
	$(CC) $(CFLAGS) -I. guard.c tests/test_guard.c -lm -o $@

clean:
	rm -f tests/test_guard
