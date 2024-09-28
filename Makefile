# compile fractal-maze-solver.c
.PHONY:all clean gdb
all:solver
solver:solver.c
	g++ -o $@ $^ -lm
clean:
	rm -f solver
gdb:solver.c
	g++ -g3 -O0 -o solver solver.c -lm
