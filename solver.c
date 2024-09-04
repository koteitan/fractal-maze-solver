#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <unordered_set>
#define MODEL_2022_SHUKE_NO3

typedef struct Global{
  struct Global *parent;
  int depth;
  int block;
  int port;
  bool operator==(const Global &other) const {
    return (depth == other.depth) && (parent == other.parent) && (block == other.block) && (port == other.port);
  }
}Global;

struct GlobalPtrEqual {
    bool operator()(const Global* g1, const Global* g2) const {
        if (!g1 || !g2) return g1 == g2;
        return *g1 == *g2;
    }
};

struct GlobalHash {
  std::size_t operator()(const Global *g) const {
    return std::hash<int>()(g->depth) ^ std::hash<Global*>()(g->parent) ^ std::hash<int>()(g->block) ^ std::hash<int>()(g->port);
  }
};

#ifdef MODEL_2022_SHUKE_NO3
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
Global start = {NULL, 0, 0, 0};
Global goal  = {NULL, 0, 2, 0};
const char *blockname[NBLOCK] = {"L", "R"};
static void print_global(Global *g){
  char out[8192]="";
  char str[8192];
  Global *local = g;
  do{
    sprintf(str, "%s%d", blockname[local->block], local->port);
    strcat(str, out);
    strcpy(out, str);
    local = local->parent;
  }while(local != NULL);
  printf("%s", out);
}
static void getnext(std::vector<Global> *vto, Global *from){
  printf("from : "); print_global(from); printf("\n");
  int b = from->block;
  int p = from->port;
  int num = nnextlocal[b][p];
  int idx = inextlocal[b][p];
  int depth = from->depth;
  Global *parent = from->parent;
  for(int i = 0; i < num; i++){
    int local = idx + i;
    int ddepth  = nextlocal[local][0];
    if(ddepth == +1){
      // go down
      Global to;
      to.depth  = ddepth + depth;
      to.port   = nextlocal[i][2];
      to.block  = nextlocal[local][1];
      to.parent = from;
      vto->push_back(to);
      printf("todn: "); print_global(&to); printf("\n");
    }else{
      continue;
      // go up
      if(parent == NULL) continue;
      Global to;
      to.depth  = ddepth + depth;
      to.port   = nextlocal[i][2];
      to.block  = parent->block;
      to.parent = parent->parent;
      vto->push_back(to);
      printf("toup: "); print_global(&to); printf("\n");
      /* nop */
    }
  }
}
#endif
static std::unordered_set<Global* , GlobalHash, GlobalPtrEqual> coldpool;
static std::unordered_set<Global* , GlobalHash, GlobalPtrEqual> hotpool;
static std::unordered_set<Global* , GlobalHash, GlobalPtrEqual> newpool;
static void print_pools(){
  printf("coldpool:\n");
  for(auto it=coldpool.begin(); it!=coldpool.end(); it++){
    Global *g = *it;
    printf("%p:{%d,%d,%d,%p}\n", g, g->depth, g->block, g->port, g->parent);
  }
  printf("hotpool:\n");
  for(auto it=hotpool.begin(); it!=hotpool.end(); it++){
    Global *g = *it;
    printf("%p:{%d,%d,%d,%p}\n", g, g->depth, g->block, g->port, g->parent);
  }
  printf("newpool:\n");
  for(auto it=newpool.begin(); it!=newpool.end(); it++){
    Global *g = *it;
    printf("%p:{%d,%d,%d,%p}\n", g, g->depth, g->block, g->port, g->parent);
  }
}

int main(int argc, char *argv[]) {
  const int maxmaxdepth = 10;
  const int maxmaxstep  = 10;
  for(int maxdepth=0; maxdepth<maxmaxdepth; maxdepth++){
    int max_reached_depth = -1;
    for(int maxstep=1; maxstep<maxmaxstep; maxstep++){
      int reached_depth = 0;
      printf("maxdepth=%d, maxstep=%d\n", maxdepth, maxstep);
      /* init game */
      hotpool.insert(&start);
      /* start game */
      for(int istep=0; istep<maxstep; istep++){
        /* take each global position from hotpool */
        bool isfound = false;

        print_pools();

        for(auto it=hotpool.begin(); it!=hotpool.end(); it++){
          Global *from = *it;
          if(from->depth > maxdepth) continue;

          std::vector<Global> tolist;
          getnext(&tolist, from);
          int nnext = tolist.size();
          printf("tolist:\n");
          for(int inext=0; inext<nnext; inext++){
            Global to = tolist[inext];
            printf("%p:{%d,%d,%d,%p}\n", &to, to.depth, to.block, to.port, to.parent);
          }

          /* take each next global position and add it to newpool */
          for(int inext=0; inext<nnext; inext++){
            Global to = tolist[inext];
            Global *pto = NULL;
            if(coldpool.find(&to) == coldpool.end() && hotpool.find(&to) == hotpool.end()){
              isfound = true;
              pto = new Global;
              *pto = to;
              newpool.insert(pto);
              if(to.depth > reached_depth) reached_depth = to.depth;
            }
            printf("%4d:", istep); print_global(pto); printf("<-"); print_global(from); printf("\n");

            if(to == goal){
              printf("goal\n");
              return EXIT_SUCCESS;
            }
          } /* for all next */

        } /* for all elements in hotpool */
        if(!isfound) break;

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

      if(reached_depth > max_reached_depth){
        max_reached_depth = reached_depth;
      }else{
        break;
      }

    } /* for all maxsteps */
  } /* for all maxdepths */
    
  return EXIT_SUCCESS;
}
