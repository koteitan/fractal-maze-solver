#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_DEPTH (100)
#define SHUKE_2022_3

typedef struct state{
  struct state *parent;
  int depth;
  int block;
  int port;
} state;

#ifdef SHUKE_2022_3
#define NBLOCK (2)
#define NPORTS (3)
  const int inextpos[NBLOCK][NPORTS] = {{0, 1, 4}, {6, 9, 11}};
  const int nnextpos[NBLOCK][NPORTS] = {{1, 3, 2}, {3, 2,  2}};
  const int nextpos[13][3] = {
  //{dD, B, P}, //pos(D, BP) -> (D  , BP)
    {+1, 0, 1}, // 0 (d, L0) -> (d+1, L1)
    {+1, 0, 2}, // 1 (d, L1) -> (d+1, L2)
    {+1, 1, 1}, // 2 (d, L1) -> (d+1, R1)
    {-1, 0, 0}, // 3 (d, L1) -> (d-1, *0)
    {+1, 1, 2}, // 4 (d, L2) -> (d+1, R2)
    {+1, 0, 1}, // 5 (d, R0) -> (d+1, L1)
    {+1, 1, 1}, // 6 (d, R0) -> (d+1, R1)
    {-1, 0, 0}, // 7 (d, R0) -> (d-1, *0)
    {-1, 0, 2}, // 8 (d, R0) -> (d-1, *2)
    {+1, 0, 2}, // 9 (d, R1) -> (d+1, L2)
    {+1, 1, 1}, //10 (d, R1) -> (d+1, R1)
    {+1, 1, 2}, //11 (d, R2) -> (d+1, R2)
    {-1, 0, 0}, //12 (d, R2) -> (d-1, *0)
  };  
#endif


void next(state **to, state *from){
  int b = from->block;
  int p = from->port;
  int num = nnextpos[b][p];
  int idx = inextpos[b][p];
  int depth = from->depth;
  state *parent = from->parent;
  for(int i = 0; i < num; i++){
    int pos = idx + i;
    int ddepth  = nextpos[pos][0];
    to[i]->depth = ddepth + depth;
    to[i]->port  = nextpos[i][2];
    if(ddepth == +1){
      to[i]->block = nextpos[pos][1];
    }else if(depth != 1){
      to[i]->block = parent->block;
    }
    
  }
}


int main(int argc, char *argv[]) {
  return EXIT_SUCCESS;
}
