#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <unordered_set>
#undef  MODEL_TEST_1
#define MODEL_2022_SNUKE_NO3

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

#ifdef MODEL_TEST_1
#define NBLOCK   (1)
#define NPORTS   (6)
#define NNEXTMAX (1)
const int inextlocal[NBLOCK][NPORTS] = {{0, 1, 2, 3, 4, 5}};
const int nnextlocal[NBLOCK][NPORTS] = {{1, 1, 1, 1, 1, 0}};
const int nextlocal[5][3] = {
//{dD, B, P}, //local(D, P) -> (D  , P)
  {+1, 0, 1}, // 0   (d, 0) -> (d+1, 1)
  {+1, 0, 2}, // 1   (d, 1) -> (d+1, 2)
  {+0, 0, 3}, // 2   (d, 2) -> (d  , 3)
  {-1, 0, 4}, // 3   (d, 3) -> (d-1, 4)
  {-1, 0, 5}, // 4   (d, 4) -> (d-1, 5)
};
/*                          d, b, p */
Global start = {NULL, 0, 0, 0};
Global goal  = {NULL, 0, 0, 5};
static const int startmaxdepth = 12;
static const int maxmaxdepth   = 13;
static const int startmaxstep  = 12;
static const int maxmaxstep    = 13;
static void print_global(Global *g){
  char out[8192]="";
  char str[8192];
  Global *local = g;
  do{
    sprintf(str, "%d", local->port);
    strcat(str, out);
    strcpy(out, str);
    local = local->parent;
  }while(local != NULL);
  printf("%s", out);
}
static void getnext(std::vector<Global> *vto, Global *from){
  //printf("from : "); print_global(from); printf("\n");
  int b = from->block;
  int p = from->port;
  int num = nnextlocal[b][p];
  int idx = inextlocal[b][p];
  int depth = from->depth;
  Global *parent = from->parent;
  for(int i = 0; i < num; i++){
    int local = idx + i;
    printf("local=%d\n", local);
    int ddepth  = nextlocal[local][0];
    printf("ddepth=%d\n", ddepth);
    if(ddepth == +1){
      // go down or stay
      Global to;
      to.depth  = ddepth + depth;
      to.port   = nextlocal[local][2];
      to.block  = nextlocal[local][1];
      to.parent = from;
      vto->push_back(to);
      printf("todn: "); print_global(&to); printf("\n");
    }if(ddepth == 0){
      // stay
      Global to;
      to.depth  = ddepth + depth;
      to.port   = nextlocal[local][2];
      to.block  = b;
      to.parent = parent;
      vto->push_back(to);
      printf("toeq: "); print_global(&to); printf("\n");
    }else{
      // go up
      if(parent == NULL) continue;
      Global to;
      to.depth  = ddepth + depth;
      to.port   = nextlocal[local][2];
      to.block  = parent->block;
      to.parent = parent->parent;
      vto->push_back(to);
      printf("toup: "); print_global(&to); printf("\n");
      /* nop */
    }
  }
}
#endif /* MODEL_TEST_1 */

#ifdef MODEL_2022_SNUKE_NO3
#define NBLOCK   (2)
#define NPORTS   (3)
#define NNEXTMAX (4)
const int inextlocal[NBLOCK][NPORTS] = {{0, 1, 4}, {6, 9, 11}};
const int nnextlocal[NBLOCK][NPORTS] = {{1, 3, 2}, {3, 2,  2}};
const int nextlocal[13][3] = {
//{dD, B, P}, //local(D, BP) -> (D  , BP)
  {+1, 0, 1}, // 0   (d, L0) -> (d+1, L1)
  {+1, 0, 2}, // 1   (d, L1) -> (d+1, L2)
  {+1, 1, 1}, // 2   (d, L1) -> (d+1, R1)
  {-1, 0, 0}, // 3   (d, L1) -> (d-1, U0)
  {+1, 1, 2}, // 4   (d, L2) -> (d+1, R2)
  {-1, 0, 1}, // 5   (d, L2) -> (d-1, U1)
  {+1, 0, 1}, // 6   (d, R0) -> (d+1, L1)
  {-1, 0, 1}, // 7   (d, R0) -> (d-1, U1)
  {-1, 0, 2}, // 8   (d, R0) -> (d-1, U2)
  {+1, 0, 2}, // 9   (d, R1) -> (d+1, L2)
  {+1, 1, 1}, //10   (d, R1) -> (d+1, R1)
  {+1, 1, 2}, //11   (d, R2) -> (d+1, R2)
  {-1, 0, 0}, //12   (d, R2) -> (d-1, U0)
}; 
/*                       d, b, p */
Global L0     = {NULL  , 0, 0, 0};
Global L0L1   = {&L0   , 1, 0, 1};
Global L0L1L1 = {&L0L1 , 2, 0, 1};
Global L0R0   = {&L0   , 1, 1, 0};
Global L2     = {NULL  , 0, 0, 2};
Global start  = L0;
Global goal   = L2;
static const int N = 17;
static const int startmaxdepth = N-1;
static const int maxmaxdepth   = N;
static const int startmaxstep  = N-1;
static const int maxmaxstep    = N;
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
  //printf("from : "); print_global(from); printf("\n");
  int b = from->block;
  int p = from->port;
  int num = nnextlocal[b][p];
  int idx = inextlocal[b][p];
  int depth = from->depth;
  Global *parent = from->parent;
  for(int i = 0; i < num; i++){
    int local = idx + i;
    int ddepth  = nextlocal[local][0];
    if(ddepth != -1){
      // go down or stay
      Global to;
      to.depth  = ddepth + depth;
      to.port   = nextlocal[local][2];
      to.block  = nextlocal[local][1];
      to.parent = from;
      vto->push_back(to);
      //printf("todn: "); print_global(&to); printf("\n");
    }else{
      // go up
      //printf("toup: local=%d\n", local);
      if(parent == NULL) continue;
      Global to;
      to.depth  = ddepth + depth;
      to.port   = nextlocal[local][2];
      to.block  = parent->block;
      to.parent = parent->parent;
      vto->push_back(to);
      //printf("toup: "); print_global(&to); printf("\n");
    }
  }
}
#endif /* MODEL_2022_SNUKE_NO3 */

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
  for(int maxdepth=startmaxdepth; maxdepth<=maxmaxdepth; maxdepth++){
    int max_reached_depth = -1;
    for(int maxstep=startmaxstep; maxstep<=maxmaxstep; maxstep++){
      int reached_depth = 0;
      printf("maxdepth=%d, maxstep=%d\n", maxdepth, maxstep);
      /* init game */
      hotpool.insert(&start);
      /* start game */
      for(int istep=0; istep<=maxstep; istep++){
        /* take each global position from hotpool */
        bool isfound = false;

        //print_pools();

        for(auto it=hotpool.begin(); it!=hotpool.end(); it++){
          Global *from = *it;
          if(from->depth > maxdepth) continue;

          std::vector<Global> tolist;
          getnext(&tolist, from);
          int nnext = tolist.size();
#if 0
          printf("tolist:\n");
          for(int inext=0; inext<nnext; inext++){
            Global to = tolist[inext];
            printf("%p:{%d,%d,%d,%p}\n", &to, to.depth, to.block, to.port, to.parent);
          }
#endif

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
              printf("%4d:", istep); print_global(pto); printf("<-"); print_global(from); printf("\n");
            }

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
