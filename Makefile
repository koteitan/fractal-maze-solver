# compile fractal-maze-solver.c
.PHONY:all clean
all:fractal-maze-solver
fractal-maze-solver:fractal-maze-solver.c
	gcc -o $@ $^ -lm
clean:
	rm -f fractal-maze-solver

