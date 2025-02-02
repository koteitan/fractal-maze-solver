.PHONY:all clean
all:solver
solver:solver.o maze_snuke_no3.o maze_koteitan.o
	g++ -o solver solver.o maze_snuke_no3.o maze_koteitan.o -lm
.cpp.o:
	g++ -c $<
clean:
	rm -f solver *.o
