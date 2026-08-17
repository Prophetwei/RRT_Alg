// ===========================================================================
//  rrt.cpp  --  optimized software RRT for comparison with rrt_con.cpp
//
//  This version applies the same algorithmic optimizations used by 01_RTL,
//  while intentionally removing hardware-only PE parallelism, arbitration
//  and pipeline timing.  Its runtime is compared with the conventional
//  software algorithm in rrt_con.cpp.
//
//  Matched to the accelerator's search algorithm:
//    * 256^3 voxel space, 16x16x16 grid index (cell = voxel>>4), with at most
//      16 nodes in each grid cell
//    * start tree root (0,0,0), goal tree root (255,255,255)
//    * each planning iteration grows START once and then GOAL once; one 24-bit
//      random word supplies x/y/z and the 1/16 opposite-root bias test
//    * nearest neighbour: fixed root-side probe order; after the first map
//      collision, three lateral probes are enabled.  If all probes are empty,
//      the search marches toward the tree's own root.  Every node in the first
//      non-empty grid cell participates in NN selection.
//    * steering vector     : the exact {sign, step} quantization of PE.sv
//    * extension           : step up to 8 voxels toward the sample; if the
//      first diagonal step is blocked, retry active axes in x/y/z order
//    * connection check    : reduced one-ring of the new node's cell in the
//      OPPOSITE tree, +/-1 voxel (Chebyshev) match
//    * insertion           : exact duplicate suppression and a 16-node
//      per-cell capacity
//
//  This is an algorithmic model, not a cycle-accurate model of SRAM/arbiters.
//
//  Build:  g++ -O3 -DNDEBUG -std=c++17 rrt.cpp -o rrt
//  Run  :  ./rrt [map] [repeats] [seed] [warmups] [save] [max_iter] [require_cert]
//  Each timed repeat resets the RNG to the same seed.  Repeats therefore
//  measure timing noise for identical work; different seeds sample the
//  algorithm distribution.
// ===========================================================================
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cstdint>
#include <chrono>
#include <algorithm>
#include <random>
#include <ctime>
#include <iomanip>
#include <numeric>
#include <array>


static const int  MAP = 256;
static const int  GRID = 16;      // MAP>>4
static const int  MAXSTEP = 8;    // successful steps when old step_count == 0..7
static const int  MAX_NODES_PER_GRID = 16;
static const long DEFAULT_MAXITER = 4000000L;

struct Node { uint8_t x, y, z; int parent; };

// ---- obstacle map (shared with the RTL: 1 = occupied) ----
static std::vector<uint8_t> occ;                 // MAP^3, index (x<<16)|(y<<8)|z
static inline bool blocked(int x, int y, int z) {
    if (x < 0 || x >= MAP || y < 0 || y >= MAP || z < 0 || z >= MAP) return true;
    return occ[(x << 16) | (y << 8) | z] != 0;
}

static bool verify_path_certificate(const std::string& map_path){
    std::string path_file=map_path;
    if (path_file.size()>=4 && path_file.substr(path_file.size()-4)==".txt")
        path_file.resize(path_file.size()-4);
    path_file += "_guaranteed_path.txt";
    std::ifstream input(path_file);
    if (!input.is_open()){
        std::cerr<<"ERROR: cannot open path certificate '"<<path_file<<"'\n";
        return false;
    }
    int x,y,z, px=-1,py=-1,pz=-1, count=0;
    while (input>>x>>y>>z){
        if (blocked(x,y,z)){
            std::cerr<<"ERROR: path certificate intersects obstacle/out-of-bounds\n";
            return false;
        }
        if (count==0){
            if (x!=0 || y!=0 || z!=0) return false;
        } else if (std::abs(x-px)+std::abs(y-py)+std::abs(z-pz)!=1){
            std::cerr<<"ERROR: path certificate is not 6-connected\n";
            return false;
        }
        px=x; py=y; pz=z; count++;
    }
    if (count!=3*(MAP-1)+1 || px!=MAP-1 || py!=MAP-1 || pz!=MAP-1){
        std::cerr<<"ERROR: path certificate has invalid length/endpoints\n";
        return false;
    }
    std::cout<<"Verified guaranteed path certificate '"<<path_file<<"'\n";
    return true;
}

// ---- one tree = flat node list + 16^3 grid of slot indices ----
struct Tree {
    std::vector<Node> node;
    std::vector<int>  cell[GRID][GRID][GRID];
    void reset() {
        node.clear();
        for (int i=0;i<GRID;i++) for (int j=0;j<GRID;j++) for (int k=0;k<GRID;k++)
            cell[i][j][k].clear();
    }
    void root(int x,int y,int z){
        node.push_back({(uint8_t)x,(uint8_t)y,(uint8_t)z,-1});
        cell[x>>4][y>>4][z>>4].push_back(0);
    }
    // Suppress exact duplicates and reject insertions into a full grid cell.
    int insert(int x,int y,int z,int parent){
        int cx=x>>4, cy=y>>4, cz=z>>4;
        std::vector<int>& bucket=cell[cx][cy][cz];
        for (int idx : bucket) {
            const Node& n=node[idx];
            if ((int)n.x==x && (int)n.y==y && (int)n.z==z) return -1;
        }
        if (bucket.size() >= MAX_NODES_PER_GRID) return -1;
        node.push_back({(uint8_t)x,(uint8_t)y,(uint8_t)z,parent});
        int idx=(int)node.size()-1;
        bucket.push_back(idx);
        return idx;
    }
};

struct GridCell { int x,y,z; };
struct Direction { int x,y,z; };

static inline bool same_cell(const GridCell& a,const GridCell& b){
    return a.x==b.x && a.y==b.y && a.z==b.z;
}

// STEP_DOWN is negative for START and positive for GOAL.  away_from_root
// mirrors probe_cell()'s flag in PE.sv.
static inline int step_grid(int g,bool active_is_start,bool away_from_root){
    bool step_high=(!active_is_start)^away_from_root;
    if (step_high) return (g==GRID-1)?GRID-1:g+1;
    return (g==0)?0:g-1;
}

static GridCell probe_cell(int probe,bool wide,const GridCell& center,
                           const GridCell& previous,bool active_is_start){
    int rx=step_grid(center.x,active_is_start,false);
    int ry=step_grid(center.y,active_is_start,false);
    int rz=step_grid(center.z,active_is_start,false);
    int ax=step_grid(center.x,active_is_start,true);
    int ay=step_grid(center.y,active_is_start,true);
    int az=step_grid(center.z,active_is_start,true);
    if (wide){
        switch (probe){
            case 1:  return {rx,ry,rz};
            case 2:  return {center.x,ry,rz};
            case 3:  return {ax,ry,rz};
            case 4:  return {rx,center.y,rz};
            case 5:  return {center.x,center.y,rz};
            case 6:  return {rx,ay,rz};
            case 7:  return {rx,ry,center.z};
            case 8:  return {center.x,ry,center.z};
            case 9:  return {rx,center.y,center.z};
            case 10: return {rx,ry,az};
            default: break;
        }
    } else {
        switch (probe){
            case 1: return {rx,ry,rz};
            case 2: return {center.x,ry,rz};
            case 3: return {rx,center.y,rz};
            case 4: return {center.x,center.y,rz};
            case 5: return {rx,ry,center.z};
            case 6: return {center.x,ry,center.z};
            case 7: return {rx,center.y,center.z};
            default: break;
        }
    }
    return {step_grid(previous.x,active_is_start,false),
            step_grid(previous.y,active_is_start,false),
            step_grid(previous.z,active_is_start,false)};
}

static inline int local_axis_score(int grid_coord,int sample_grid,
                                   int node_offset,int sample_offset){
    if (grid_coord<sample_grid) return 15-node_offset;
    if (grid_coord>sample_grid) return node_offset;
    return std::abs(node_offset-sample_offset);
}

// Search cells in PE.sv's order and stop at the first non-empty cell.  The
// software model compares every node in that cell instead of emulating slots.
static int nearest(const Tree& t,int sx,int sy,int sz,bool active_is_start,
                   bool wide,uint64_t& checks){
    GridCell center{sx>>4,sy>>4,sz>>4};
    auto scan_cell=[&](const GridCell& grid)->int {
        const std::vector<int>& bucket=t.cell[grid.x][grid.y][grid.z];
        if (bucket.empty()) return -1;
        int best=-1, bestd=63;
        for (int idx : bucket){
            const Node& n=t.node[idx];
            checks++;
            int d=local_axis_score(grid.x,center.x,(int)n.x&15,sx&15)
                 +local_axis_score(grid.y,center.y,(int)n.y&15,sy&15)
                 +local_axis_score(grid.z,center.z,(int)n.z&15,sz&15);
            if (d<bestd){ bestd=d; best=idx; }
        }
        return best;
    };

    int root_march_probe=wide?11:8;
    GridCell cursor=center;
    for (int probe=0;probe<root_march_probe;probe++){
        if (probe!=0)
            cursor=probe_cell(probe,wide,center,cursor,active_is_start);
        int best=scan_cell(cursor);
        if (best>=0) return best;
    }

    while (true){
        GridCell next=probe_cell(root_march_probe,wide,center,cursor,
                                 active_is_start);
        if (same_cell(next,cursor)) return -1; // root slot makes this unreachable
        cursor=next;
        int best=scan_cell(cursor);
        if (best>=0) return best;
    }
}

// Exact VECTOR_QUANT comparisons from PE.sv.
static Direction steer(int sx,int sy,int sz,const Node& n){
    int dx=sx-(int)n.x, dy=sy-(int)n.y, dz=sz-(int)n.z;
    int adx=std::abs(dx), ady=std::abs(dy), adz=std::abs(dz);
    return {(adx>(ady>>1))?((dx<0)?-1:1):0,
            (ady>(adx>>1))?((dy<0)?-1:1):0,
            (adz>(adx>>1) && adz>(ady>>1))?((dz<0)?-1:1):0};
}

// ---- connection results ----
static int start_conn=-1, goal_conn=-1;

// ---- reduced one-ring connection check against the opposite tree ---------
//  returns true if the new node connects (records the two endpoint indices)
static bool connect(const Tree& other, bool active_is_start,
                    int nx,int ny,int nz, int new_idx, uint64_t& checks){
    int cx=nx>>4, cy=ny>>4, cz=nz>>4;
    int ox=nx&15, oy=ny&15, oz=nz&15;
    int xl=(ox==0 &&cx>0)?cx-1:cx, xh=(ox==15&&cx<GRID-1)?cx+1:cx;
    int yl=(oy==0 &&cy>0)?cy-1:cy, yh=(oy==15&&cy<GRID-1)?cy+1:cy;
    int zl=(oz==0 &&cz>0)?cz-1:cz, zh=(oz==15&&cz<GRID-1)?cz+1:cz;
    for (int gx=xl;gx<=xh;gx++) for(int gy=yl;gy<=yh;gy++) for(int gz=zl;gz<=zh;gz++)
        for (int idx : other.cell[gx][gy][gz]){
            checks++;
            const Node& m=other.node[idx];
            if (std::abs(nx-(int)m.x)<=1 && std::abs(ny-(int)m.y)<=1 && std::abs(nz-(int)m.z)<=1){
                if (active_is_start){ start_conn=new_idx; goal_conn=idx; }
                else                { goal_conn =new_idx; start_conn=idx; }
                return true;
            }
        }
    return false;
}

struct Candidate {
    bool valid=false;
    int x=0,y=0,z=0,parent=-1;
};

// One tree-growth transaction: RANDOM -> NN -> VECTOR -> EXTEND.  Insertion and
// connection checking are deliberately kept outside this function.
static Candidate grow_candidate(const Tree& tree,bool active_is_start,bool wide,
                                std::mt19937& rng,uint64_t& nn_checks,
                                uint64_t& collision_checks,bool& saw_obstacle){
    saw_obstacle=false;
    uint32_t word=rng();
    int sx,sy,sz;
    if ((word&0xFu)==0){
        int other_root=active_is_start?MAP-1:0;
        sx=sy=sz=other_root;
    } else {
        sx=(int)(word&0xFFu);
        sy=(int)((word>>8)&0xFFu);
        sz=(int)((word>>16)&0xFFu);
    }

    int ni=nearest(tree,sx,sy,sz,active_is_start,wide,nn_checks);
    if (ni<0) return {};
    const Node& nn=tree.node[ni];
    Direction base=steer(sx,sy,sz,nn);
    int active_axes=(base.x!=0)+(base.y!=0)+(base.z!=0);
    if (active_axes==0) return {};

    std::array<Direction,4> directions{};
    int direction_count=0;
    directions[direction_count++]=base;
    if (active_axes>1){
        if (base.x!=0) directions[direction_count++]={base.x,0,0};
        if (base.y!=0) directions[direction_count++]={0,base.y,0};
        if (base.z!=0) directions[direction_count++]={0,0,base.z};
    }

    for (int d=0;d<direction_count;d++){
        int lx=(int)nn.x, ly=(int)nn.y, lz=(int)nn.z;
        int committed=0;
        for (int step=0;step<MAXSTEP;step++){
            int px=lx+directions[d].x;
            int py=ly+directions[d].y;
            int pz=lz+directions[d].z;
            if (px<0 || px>=MAP || py<0 || py>=MAP || pz<0 || pz>=MAP)
                break;
            collision_checks++;
            if (blocked(px,py,pz)){
                saw_obstacle=true;
                break;
            }
            lx=px; ly=py; lz=pz; committed++;
        }
        if (committed!=0)
            return {true,lx,ly,lz,ni};
    }
    return {};
}

struct RunStat {
    bool found;
    double wall_sec, cpu_sec;
    long iters;
    size_t s_nodes, g_nodes;
    int path_len;
    uint64_t nn_checks, connection_checks, collision_checks;
};

static double process_cpu_seconds(){
    timespec ts{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0)
        return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
    return (double)std::clock() / CLOCKS_PER_SEC;
}

static double mean(const std::vector<double>& values){
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

static double median(std::vector<double> values){
    std::sort(values.begin(), values.end());
    size_t n=values.size();
    return (n&1) ? values[n/2] : (values[n/2-1]+values[n/2])*0.5;
}

static RunStat plan_once(Tree& S,Tree& G,unsigned seed,long max_iterations){
    S.reset(); G.reset();
    S.root(0,0,0);
    G.root(MAP-1,MAP-1,MAP-1);
    start_conn=goal_conn=-1;
    std::mt19937 rng(seed);
    bool obstacle_seen=false;

    uint64_t nn_checks=0, connection_checks=0, collision_checks=0;
    auto t0=std::chrono::steady_clock::now();
    double c0=process_cpu_seconds();
    bool found=false; long it=0;
    auto grow_tree=[&](bool active_is_start)->bool {
        Tree& own=active_is_start?S:G;
        Tree& other=active_is_start?G:S;
        bool saw_obstacle=false;
        Candidate cand=grow_candidate(own,active_is_start,obstacle_seen,rng,
                                      nn_checks,collision_checks,saw_obstacle);
        obstacle_seen=obstacle_seen||saw_obstacle;
        if (!cand.valid) return false;
        int idx=own.insert(cand.x,cand.y,cand.z,cand.parent);
        if (idx<0) return false; // exact duplicate
        return connect(other,active_is_start,cand.x,cand.y,cand.z,idx,
                       connection_checks);
    };

    while (!found && it<max_iterations){
        it++;
        found=grow_tree(true);
        if (!found) found=grow_tree(false);
    }
    double c1=process_cpu_seconds();
    auto t1=std::chrono::steady_clock::now();

    RunStat r;
    r.found=found;
    r.wall_sec=std::chrono::duration<double>(t1-t0).count();
    r.cpu_sec=c1-c0;
    r.iters=it;
    r.s_nodes=S.node.size(); r.g_nodes=G.node.size();
    r.path_len=0;
    if (found){                                                    // count START..GOAL waypoints
        for (int c=start_conn; c!=-1; c=S.node[c].parent) r.path_len++;
        for (int c=goal_conn;  c!=-1; c=G.node[c].parent) r.path_len++;
    }
    r.nn_checks=nn_checks;
    r.connection_checks=connection_checks;
    r.collision_checks=collision_checks;
    return r;
}

static bool build_solution_path(const Tree& S,const Tree& G,std::vector<Node>& path){
    path.clear();
    if (start_conn<0 || goal_conn<0) return false;
    int guard=0;
    for (int c=start_conn;c!=-1;c=S.node[c].parent){
        if (c<0 || c>=(int)S.node.size() || guard++>(int)S.node.size()) return false;
        path.push_back(S.node[c]);
    }
    std::reverse(path.begin(), path.end());
    guard=0;
    for (int c=goal_conn;c!=-1;c=G.node[c].parent){
        if (c<0 || c>=(int)G.node.size() || guard++>(int)G.node.size()) return false;
        path.push_back(G.node[c]);
    }
    return true;
}

// Same endpoint, straight-segment, length and voxel checks as PATTERN.sv.
static bool validate_solution(const Tree& S,const Tree& G){
    std::vector<Node> path;
    if (!build_solution_path(S,G,path) || path.size()<2) return false;
    if (path.front().x!=0 || path.front().y!=0 || path.front().z!=0) return false;
    if (path.back().x!=MAP-1 || path.back().y!=MAP-1 || path.back().z!=MAP-1) return false;
    for (const Node& n : path)
        if (blocked((int)n.x,(int)n.y,(int)n.z)) return false;

    for (size_t i=0;i+1<path.size();i++){
        int ax=path[i].x, ay=path[i].y, az=path[i].z;
        int bx=path[i+1].x, by=path[i+1].y, bz=path[i+1].z;
        int dx=std::abs(bx-ax), dy=std::abs(by-ay), dz=std::abs(bz-az);
        int span=std::max(dx,std::max(dy,dz));
        if (span>MAXSTEP) return false;
        if (span!=0 && !((dx==0 || dx==span) &&
                         (dy==0 || dy==span) &&
                         (dz==0 || dz==span))) return false;
        int stepx=(bx>ax)?1:((bx<ax)?-1:0);
        int stepy=(by>ay)?1:((by<ay)?-1:0);
        int stepz=(bz>az)?1:((bz<az)?-1:0);
        for (int k=1;k<=span;k++)
            if (blocked(ax+stepx*k,ay+stepy*k,az+stepz*k)) return false;
    }
    return true;
}

// ---- reconstruct START..GOAL path and write tree/ files ----
static void extract_and_save(const Tree& S, const Tree& G){
    std::vector<Node> path;
    if (!build_solution_path(S,G,path)){ std::cout<<"no connection to extract\n"; return; }

    std::ofstream pf("tree/path_3D");
    for (auto&n:path) pf<<(int)n.x<<" "<<(int)n.y<<" "<<(int)n.z<<"\n";
    std::ofstream sf("tree/start_nodes_3D");
    for (auto&n:S.node) sf<<(int)n.x<<" "<<(int)n.y<<" "<<(int)n.z<<"\n";
    std::ofstream gf("tree/goal_nodes_3D");
    for (auto&n:G.node) gf<<(int)n.x<<" "<<(int)n.y<<" "<<(int)n.z<<"\n";
    std::cout<<"Path length: "<<path.size()<<" nodes  (written to tree/path_3D)\n";
}

static bool same_work(const RunStat& a, const RunStat& b){
    return a.found==b.found && a.iters==b.iters &&
           a.s_nodes==b.s_nodes && a.g_nodes==b.g_nodes &&
           a.path_len==b.path_len && a.nn_checks==b.nn_checks &&
           a.connection_checks==b.connection_checks &&
           a.collision_checks==b.collision_checks;
}

int main(int argc, char** argv){
    std::string map_path = (argc>1) ? argv[1] : "map/map_0.txt";
    int runs = (argc>2) ? std::atoi(argv[2]) : 1;
    unsigned seed = (argc>3) ? (unsigned)std::strtoul(argv[3],nullptr,10) : 42u;
    int warmups = (argc>4) ? std::atoi(argv[4]) : 0;
    bool save_output = (argc>5) ? std::atoi(argv[5])!=0 : true;
    long max_iterations = (argc>6) ? std::strtol(argv[6],nullptr,10) : DEFAULT_MAXITER;
    bool require_certificate = (argc>7) ? std::atoi(argv[7])!=0 : false;
    if (runs<1 || warmups<0 || max_iterations<1){
        std::cerr<<"Usage: ./rrt [map] [repeats>=1] [seed] [warmups>=0] "
                 <<"[save_output=0|1] [max_iterations>=1] [require_certificate=0|1]\n";
        return 2;
    }

    // ---- load obstacle map ("none" is the RTL +NOMAP equivalent) ----
    occ.assign((size_t)MAP*MAP*MAP, 0);
    if (map_path=="none"){
        std::cout<<"Empty map (no obstacles)\n";
        if (require_certificate){
            std::cerr<<"ERROR: an empty map has no path-certificate file\n";
            return 4;
        }
    } else {
        auto lt0=std::chrono::high_resolution_clock::now();
        std::ifstream f(map_path);
        if (!f.is_open()){ std::cerr<<"ERROR: cannot open map '"<<map_path<<"'\n"; return 1; }
        long x,y,z,n=0;
        while (f>>x>>y>>z){
            if (x>=0&&x<MAP&&y>=0&&y<MAP&&z>=0&&z<MAP){
                occ[(x<<16)|(y<<8)|z]=1;
                n++;
            }
        }
        auto lt1=std::chrono::high_resolution_clock::now();
        std::cout<<"Map '"<<map_path<<"' : "<<n<<" obstacle voxels, load "
                 <<std::chrono::duration<double>(lt1-lt0).count()<<" s\n";
        if (require_certificate && !verify_path_certificate(map_path)) return 4;
    }

    Tree S, G;
    for (int w=0; w<warmups; w++){
        (void)plan_once(S,G,seed,max_iterations);
    }

    std::vector<double> wall_times, cpu_times;
    double it_sum=0, sn=0, gn=0, pl_sum=0;
    double nn_sum=0, conn_sum=0, collision_sum=0;
    int successes=0;
    bool deterministic=true;
    RunStat first{}, last{};
    std::cout<<std::setprecision(12);
    std::cout<<"\n--- Planning ("<<runs<<" timed repeat"<<(runs>1?"s":"")
             <<", fixed seed "<<seed<<") ---\n";
    for (int r=0;r<runs;r++){
        last=plan_once(S,G,seed,max_iterations);
        if (last.found && !validate_solution(S,G)){
            std::cerr<<"ERROR: generated path failed PATTERN-equivalent validation\n";
            return 5;
        }
        if (r==0) first=last;
        else deterministic = deterministic && same_work(first,last);
        wall_times.push_back(last.wall_sec);
        cpu_times.push_back(last.cpu_sec);
        successes += last.found ? 1 : 0;
        it_sum+=last.iters; sn+=last.s_nodes; gn+=last.g_nodes;
        pl_sum+=last.path_len; nn_sum+=last.nn_checks;
        conn_sum+=last.connection_checks; collision_sum+=last.collision_checks;
        std::cout<<"RUN model=opt repeat="<<r<<" seed="<<seed
                 <<" found="<<(last.found?1:0)
                 <<" wall_sec="<<last.wall_sec
                 <<" cpu_sec="<<last.cpu_sec
                 <<" iterations="<<last.iters
                 <<" nodes="<<(last.s_nodes+last.g_nodes)
                 <<" start_nodes="<<last.s_nodes<<" goal_nodes="<<last.g_nodes
                 <<" path_nodes="<<last.path_len
                 <<" nn_checks="<<last.nn_checks
                 <<" connection_checks="<<last.connection_checks
                 <<" collision_checks="<<last.collision_checks<<"\n";
    }

    double wall_mean=mean(wall_times), wall_median=median(wall_times);
    double cpu_mean=mean(cpu_times), cpu_median=median(cpu_times);
    double mean_nodes=(sn+gn)/runs, mean_path=pl_sum/runs;
    double tree_kb=mean_nodes*5.0/1024.0 + 8.0;
    double path_kb=mean_path*3.0/1024.0;
    std::cout<<"\n--- CPU summary ---\n";
    std::cout<<"timed repeats  : "<<runs<<" (warmups "<<warmups<<")\n";
    std::cout<<"successes      : "<<successes<<"/"<<runs<<"\n";
    std::cout<<"deterministic  : "<<(deterministic?"yes":"NO")<<"\n";
    std::cout<<"wall mean/med  : "<<wall_mean<<" / "<<wall_median<<" s\n";
    std::cout<<"CPU mean/med   : "<<cpu_mean<<" / "<<cpu_median<<" s\n";
    std::cout<<"mean iterations: "<<(it_sum/runs)<<"\n";
    std::cout<<"mean nodes     : "<<mean_nodes<<" (start "<<sn/runs<<" / goal "<<gn/runs<<")\n";
    std::cout<<"tree_mem(KB)   : "<<tree_kb<<"\n";
    std::cout<<"path_mem(KB)   : "<<path_kb<<"\n";
    std::cout<<"SUMMARY model=opt runs="<<runs<<" success="<<successes
             <<" deterministic="<<(deterministic?1:0)
             <<" wall_mean="<<wall_mean<<" wall_median="<<wall_median
             <<" cpu_mean="<<cpu_mean<<" cpu_median="<<cpu_median
             <<" iterations="<<(it_sum/runs)<<" nodes="<<mean_nodes
             <<" path_nodes="<<mean_path
             <<" nn_checks="<<(nn_sum/runs)
             <<" connection_checks="<<(conn_sum/runs)
             <<" collision_checks="<<(collision_sum/runs)
             <<" tree_mem_kb="<<tree_kb<<" path_mem_kb="<<path_kb<<"\n";

    if (save_output){
        if (last.found) extract_and_save(S,G);
        else std::cout<<"last run failed to connect.\n";
    }
    return 0;
}
