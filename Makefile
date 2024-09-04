# compile fractal-maze-solver.c
.PHONY:all clean
all:solver
solver:solver.c
	g++ -o $@ $^ -lm -O0 -g3
clean:
	rm -f fractal-maze-solver

