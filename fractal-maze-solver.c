#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unordered_set>
#define SHUKE_2022_3

typedef struct Global{
  struct Global *parent;
  int depth;
  int block;
  int port;
  bool operator==(const Global &other) const {
    return (depth == other.depth) && (parent == other.parent) && (block == other.block) && (port == other.port);
  }
}Global;

namespace std {
  template <>
  struct hash<Global> {
    std::size_t operator()(const Global &g) const {
      return std::hash<int>()(g.depth) ^ std::hash<Global*>()(g.parent) ^ std::hash<int>()(g.block) ^ std::hash<int>()(g.port);
    }
  };
}

#ifdef SHUKE_2022_3
#define NBLOCK   (2)
#define NPORTS   (3)
#define NNEXTMAX (4)
const int inextlocal[NBLOCK][NPORTS] = {{0, 1, 4}, {6, 9, 11}};
const int nnextlocal[NBLOCK][NPORTS] = {{1, 3, 2}, {3, 2,  2}};
const int nextlocal[13][3] = {
//{dD, B, P}, //local(D, BP) -> (D  , BP)
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
/*                          d, b, p */
const Global start = {NULL, 0, 0, 0};
const Global goal  = {NULL, 0, 2, 0};
const char *blockname[NBLOCK] = {"L", "R"};
static void print_global(Global *g){
  Global *local = g;
  do{
    printf("%s%d", blockname[local->block], local->port);
    local = local->parent;
  }while(local != NULL);
  printf("\n");
}
static void next(int *nto, Global **to, Global *from){
  int b = from->block;
  int p = from->port;
  *nto    = nnextlocal[b][p];
  int idx = inextlocal[b][p];
  int depth = from->depth;
  Global *parent = from->parent;
  for(int i = 0; i < *nto; i++){
    int local = idx + i;
    int ddepth  = nextlocal[local][0];
    if(ddepth == +1){
      to[i] = new Global;
      to[i]->depth = ddepth + depth;
      to[i]->port  = nextlocal[i][2];
      to[i]->block = nextlocal[local][1];
      to[i]->parent = from;
    }else if(depth != 0){
      to[i] = new Global;
      to[i]->depth = ddepth + depth;
      to[i]->port  = nextlocal[i][2];
      to[i]->block  = parent->block;
      to[i]->parent = parent->parent;
    }else{
      /* nop */
    }
  }
}
#endif

int main(int argc, char *argv[]) {
  const int maxmaxdepth = 10;
  const int maxmaxstep  = 10;
  for(int maxdepth=0; maxdepth<maxmaxdepth; maxdepth++){
    for(int maxstep=0; maxstep<maxmaxstep; maxstep++){
      printf("maxdepth=%d, maxstep=%d\n", maxdepth, maxstep);
      /* init game */
      std::unordered_set<Global> coldpool;
      std::unordered_set<Global> hotpool;
      std::unordered_set<Global> newpool;
      hotpool.insert(start);
      /* start game */
      for(int istep=0; istep<maxstep; istep++){
        /* take each global position from hotpool */
        for(auto it=hotpool.begin(); it!=hotpool.end(); it++){
          Global from = *it;
          if(from.depth >= maxdepth) continue;

          Global **tolist = new Global*[NNEXTMAX];
          int nnext = 0;
          next(&nnext, tolist, &from);

          /* take each next global position and add it to newpool */
          for(int inext=0; inext<nnext; inext++){
            Global *to = tolist[inext];
            if(coldpool.find(*to) == coldpool.end() && hotpool.find(*to) == hotpool.end()){
              newpool.insert(*to);
            }
            printf("%4d:", istep); print_global(to); printf("->"); print_global(&from); printf("\n");

            if(*to == goal){
              printf("goal\n");
              return EXIT_SUCCESS;
            }
          } /* for all next */

        } /* for all elements in hotpool */

        /* move new -> cold */
        for(auto it=hotpool.begin(); it!=hotpool.end(); it++){
          coldpool.insert(*it);
        }
        hotpool.clear();

        /* move new -> hot */
        for(auto it=newpool.begin(); it!=newpool.end(); it++){
          hotpool.insert(*it);
        }
        newpool.clear();

      } /* for all steps */

      /* game over */
      printf("maxdepth=%d, maxstep=%d was unsolvable.\n", maxdepth, maxmaxstep);
      
      /* clear all pools */
      hotpool.clear();
      coldpool.clear();

    } /* for all maxsteps */
  } /* for all maxdepths */
    
  return EXIT_SUCCESS;
}





















































