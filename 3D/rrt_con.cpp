// ===========================================================================
//  rrt_con.cpp  --  CONVENTIONAL dual-tree RRT (baseline for comparison)
//
//  Textbook, un-accelerated version, to compare against the optimized
//  (grid / Manhattan / extension) model in rrt.cpp:
//     * two trees (start root (0,0,0), goal root (255,255,255))
//     * nearest neighbour : LINEAR scan over the WHOLE tree, L2 distance
//                           sqrt(dx^2+dy^2+dz^2)  (argmin uses squared L2)
//     * steer             : new = nearest + round(unit vector nearest->sample)
//                           i.e. a SINGLE unit step, NO branch extension
//     * collision         : new node must be in-bounds and obstacle-free
//     * connection        : LINEAR scan the OPPOSITE tree for any node within
//                           L2 distance 1  (dx^2+dy^2+dz^2 <= 1)
//
//  The 256^3 map, roots, 1/16 seed-snap and balanced growth match rrt.cpp.
//  Planning differences include linear-vs-grid NN, L2-vs-Manhattan,
//  single-step-vs-extension, uncapped-vs-8-per-cell storage, and
//  linear-vs-reduced-ring connection.
//
//  Build:  g++ -O3 -DNDEBUG -std=c++17 rrt_con.cpp -o rrt_con
//  Run  :  ./rrt_con [map] [repeats] [seed] [warmups] [save] [max_iter] [require_cert]
//  Each timed repeat resets the RNG to the same seed, matching rrt.cpp.
// ===========================================================================
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cstdint>
#include <climits>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <random>
#include <ctime>
#include <iomanip>
#include <numeric>


static const int  MAP = 256;
static const long DEFAULT_MAXITER = 4000000L;

struct Node { int x, y, z; int parent; };

static std::vector<uint8_t> occ;                 // MAP^3, (x<<16)|(y<<8)|z
static inline bool blocked(int x, int y, int z) {
    if (x < 0 || x >= MAP || y < 0 || y >= MAP || z < 0 || z >= MAP) return true;
    return occ[((size_t)x << 16) | (y << 8) | z] != 0;
}

static bool verify_path_certificate(const std::string& map_path) {
    std::string path_file=map_path;
    if (path_file.size()>=4 && path_file.substr(path_file.size()-4)==".txt")
        path_file.resize(path_file.size()-4);
    path_file += "_guaranteed_path.txt";
    std::ifstream input(path_file);
    if (!input.is_open()) {
        std::cerr << "ERROR: cannot open path certificate '" << path_file << "'\n";
        return false;
    }
    int x,y,z, px=-1,py=-1,pz=-1, count=0;
    while (input>>x>>y>>z) {
        if (blocked(x,y,z)) {
            std::cerr << "ERROR: path certificate intersects obstacle/out-of-bounds\n";
            return false;
        }
        if (count==0) {
            if (x!=0 || y!=0 || z!=0) return false;
        } else if (std::abs(x-px)+std::abs(y-py)+std::abs(z-pz)!=1) {
            std::cerr << "ERROR: path certificate is not 6-connected\n";
            return false;
        }
        px=x; py=y; pz=z; count++;
    }
    if (count!=3*(MAP-1)+1 || px!=MAP-1 || py!=MAP-1 || pz!=MAP-1) {
        std::cerr << "ERROR: path certificate has invalid length/endpoints\n";
        return false;
    }
    std::cout << "Verified guaranteed path certificate '" << path_file << "'\n";
    return true;
}

// squared L2 (monotonic with L2 -> same argmin, no sqrt in the hot loop)
static inline long d2(int ax, int ay, int az, const Node& b) {
    long dx = ax - b.x, dy = ay - b.y, dz = az - b.z;
    return dx*dx + dy*dy + dz*dz;
}

// ---- LINEAR nearest neighbour over the whole tree (L2) ----
static int nearest(const std::vector<Node>& t, int sx, int sy, int sz,
                   uint64_t& checks) {
    int best = 0; long bestd = LONG_MAX;
    for (int i = 0; i < (int)t.size(); i++) {
        checks++;
        long d = d2(sx, sy, sz, t[i]);
        if (d < bestd) { bestd = d; best = i; }
    }
    return best;
}

static int start_conn = -1, goal_conn = -1;

// ---- LINEAR connection check : any opposite node within L2 distance 1 ----
static bool connect(const std::vector<Node>& other, bool active_is_start,
                    int nx, int ny, int nz, int new_idx, uint64_t& checks) {
    for (int j = 0; j < (int)other.size(); j++) {
        checks++;
        if (d2(nx, ny, nz, other[j]) <= 1) {           // within distance 1
            if (active_is_start) { start_conn = new_idx; goal_conn = j; }
            else                 { goal_conn  = new_idx; start_conn = j; }
            return true;
        }
    }
    return false;
}

struct RunStat {
    bool found;
    double wall_sec, cpu_sec;
    long iters;
    size_t s_nodes, g_nodes;
    int path_len;
    uint64_t nn_checks, connection_checks, collision_checks;
};

static double process_cpu_seconds() {
    timespec ts{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) == 0)
        return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
    return (double)std::clock() / CLOCKS_PER_SEC;
}

static double mean(const std::vector<double>& values) {
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

static double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    size_t n = values.size();
    return (n & 1) ? values[n/2] : (values[n/2-1] + values[n/2]) * 0.5;
}

static RunStat plan_once(std::vector<Node>& S, std::vector<Node>& G,
                         std::mt19937& rng, long max_iterations) {
    S.clear(); G.clear();
    S.push_back({0, 0, 0, -1});
    G.push_back({MAP-1, MAP-1, MAP-1, -1});
    start_conn = goal_conn = -1;
    std::uniform_int_distribution<int> u8(0, MAP-1), u4(0, 15);

    uint64_t nn_checks=0, connection_checks=0, collision_checks=0;
    auto t0 = std::chrono::steady_clock::now();
    double c0 = process_cpu_seconds();
    bool found = false; long it = 0;
    for (; it < max_iterations && !found; ++it) {
        bool active_is_start = (S.size() <= G.size());          // grow the smaller tree
        std::vector<Node>& A = active_is_start ? S : G;
        std::vector<Node>& O = active_is_start ? G : S;

        // sample (1/16 seed snap toward the OTHER root)
        int sx, sy, sz;
        if (u4(rng) == 0) { int s = active_is_start ? (MAP-1) : 0; sx = sy = sz = s; }
        else              { sx = u8(rng); sy = u8(rng); sz = u8(rng); }

        int ni = nearest(A, sx, sy, sz, nn_checks);             // LINEAR NN
        const Node nn = A[ni];

        // steer : single UNIT-VECTOR step toward the sample (no extension)
        double dx = sx - nn.x, dy = sy - nn.y, dz = sz - nn.z;
        double len = std::sqrt(dx*dx + dy*dy + dz*dz);          // L2 norm
        if (len < 1e-9) continue;                               // sample == nearest
        int stepx = (int)std::lround(dx / len);
        int stepy = (int)std::lround(dy / len);
        int stepz = (int)std::lround(dz / len);                 // each in {-1,0,1}
        if (stepx == 0 && stepy == 0 && stepz == 0) continue;
        int nx = nn.x + stepx, ny = nn.y + stepy, nz = nn.z + stepz;
        collision_checks++;
        if (blocked(nx, ny, nz)) continue;                      // collision

        A.push_back({nx, ny, nz, ni});
        int idx = (int)A.size() - 1;
        if (connect(O, active_is_start, nx, ny, nz, idx, connection_checks)) found = true;
    }
    double c1 = process_cpu_seconds();
    auto t1 = std::chrono::steady_clock::now();

    RunStat r;
    r.found = found;
    r.wall_sec = std::chrono::duration<double>(t1 - t0).count();
    r.cpu_sec = c1 - c0;
    r.iters = it;
    r.s_nodes = S.size(); r.g_nodes = G.size();
    r.path_len = 0;

    if (found) {
        for (int c = start_conn; c != -1; c = S[c].parent) r.path_len++;
        for (int c = goal_conn;  c != -1; c = G[c].parent) r.path_len++;
    }
    r.nn_checks=nn_checks;
    r.connection_checks=connection_checks;
    r.collision_checks=collision_checks;
    return r;
}

static void extract_and_save(const std::vector<Node>& S, const std::vector<Node>& G) {
    if (start_conn < 0 || goal_conn < 0) return;
    std::vector<Node> path;
    for (int c = start_conn; c != -1; c = S[c].parent) path.push_back(S[c]);
    std::reverse(path.begin(), path.end());
    for (int c = goal_conn; c != -1; c = G[c].parent) path.push_back(G[c]);
    std::ofstream pf("tree_con/path_3D");
    for (auto& n : path) pf << n.x << " " << n.y << " " << n.z << "\n";
    std::ofstream sf("tree_con/start_nodes_3D");
    for (auto& n : S) sf << n.x << " " << n.y << " " << n.z << "\n";
    std::ofstream gf("tree_con/goal_nodes_3D");
    for (auto& n : G) gf << n.x << " " << n.y << " " << n.z << "\n";
}

static bool same_work(const RunStat& a, const RunStat& b) {
    return a.found==b.found && a.iters==b.iters &&
           a.s_nodes==b.s_nodes && a.g_nodes==b.g_nodes &&
           a.path_len==b.path_len && a.nn_checks==b.nn_checks &&
           a.connection_checks==b.connection_checks &&
           a.collision_checks==b.collision_checks;
}

int main(int argc, char** argv) {
    std::string map_path = (argc > 1) ? argv[1] : "map/map_0.txt";
    int runs = (argc > 2) ? std::atoi(argv[2]) : 1;
    unsigned seed = (argc > 3) ? (unsigned)std::strtoul(argv[3], nullptr, 10) : 12345u;
    int warmups = (argc > 4) ? std::atoi(argv[4]) : 0;
    bool save_output = (argc > 5) ? std::atoi(argv[5]) != 0 : true;
    long max_iterations = (argc > 6) ? std::strtol(argv[6], nullptr, 10) : DEFAULT_MAXITER;
    bool require_certificate = (argc > 7) ? std::atoi(argv[7]) != 0 : false;
    if (runs < 1 || warmups < 0 || max_iterations < 1) {
        std::cerr << "Usage: ./rrt_con [map] [repeats>=1] [seed] [warmups>=0] "
                  << "[save_output=0|1] [max_iterations>=1] [require_certificate=0|1]\n";
        return 2;
    }

    occ.assign((size_t)MAP*MAP*MAP, 0);
    if (map_path != "none") {
        auto lt0 = std::chrono::high_resolution_clock::now();
        std::ifstream f(map_path);
        if (!f.is_open()) { std::cerr << "ERROR: cannot open map '" << map_path << "'\n"; return 1; }
        long x, y, z, n = 0;
        while (f >> x >> y >> z)
            if (x>=0&&x<MAP&&y>=0&&y<MAP&&z>=0&&z<MAP) { occ[((size_t)x<<16)|(y<<8)|z]=1; n++; }
        auto lt1 = std::chrono::high_resolution_clock::now();
        std::cout << "Map '" << map_path << "' : " << n << " obstacle voxels, load "
                  << std::chrono::duration<double>(lt1-lt0).count() << " s\n";
    } else {
        std::cout << "Empty map (no obstacles)\n";
    }
    if (require_certificate && !verify_path_certificate(map_path)) return 4;

    std::vector<Node> S, G;
    for (int w = 0; w < warmups; w++) {
        std::mt19937 warm_rng(seed);
        (void)plan_once(S, G, warm_rng, max_iterations);
    }

    std::vector<double> wall_times, cpu_times;
    double it_sum=0, sn=0, gn=0, pl_sum=0;
    double nn_sum=0, conn_sum=0, collision_sum=0;
    int successes=0;
    bool deterministic=true;
    RunStat first{}, last{};
    std::cout << std::setprecision(12);
    std::cout << "\n--- CONVENTIONAL RRT (" << runs << " timed repeat"
              << (runs>1?"s":"") << ", fixed seed " << seed << ") ---\n";
    for (int r = 0; r < runs; r++) {
        std::mt19937 run_rng(seed);
        last=plan_once(S,G,run_rng,max_iterations);
        if (r==0) first=last;
        else deterministic = deterministic && same_work(first,last);
        wall_times.push_back(last.wall_sec);
        cpu_times.push_back(last.cpu_sec);
        successes += last.found ? 1 : 0;
        it_sum+=last.iters; sn+=last.s_nodes; gn+=last.g_nodes;
        pl_sum+=last.path_len; nn_sum+=last.nn_checks;
        conn_sum+=last.connection_checks; collision_sum+=last.collision_checks;
        std::cout << "RUN model=con repeat=" << r << " seed=" << seed
                  << " found=" << (last.found?1:0)
                  << " wall_sec=" << last.wall_sec
                  << " cpu_sec=" << last.cpu_sec
                  << " iterations=" << last.iters
                  << " nodes=" << (last.s_nodes+last.g_nodes)
                  << " start_nodes=" << last.s_nodes << " goal_nodes=" << last.g_nodes
                  << " path_nodes=" << last.path_len
                  << " nn_checks=" << last.nn_checks
                  << " connection_checks=" << last.connection_checks
                  << " collision_checks=" << last.collision_checks << "\n";
    }

    double wall_mean=mean(wall_times), wall_median=median(wall_times);
    double cpu_mean=mean(cpu_times), cpu_median=median(cpu_times);
    double mean_nodes=(sn+gn)/runs, mean_path=pl_sum/runs;
    double tree_kb=mean_nodes*5.0/1024.0;
    double path_kb=mean_path*3.0/1024.0;
    std::cout << "\n--- CPU summary (conventional) ---\n";
    std::cout << "timed repeats  : " << runs << " (warmups " << warmups << ")\n";
    std::cout << "successes      : " << successes << "/" << runs << "\n";
    std::cout << "deterministic  : " << (deterministic?"yes":"NO") << "\n";
    std::cout << "wall mean/med  : " << wall_mean << " / " << wall_median << " s\n";
    std::cout << "CPU mean/med   : " << cpu_mean << " / " << cpu_median << " s\n";
    std::cout << "mean iterations: " << (it_sum/runs) << "\n";
    std::cout << "mean nodes     : " << mean_nodes << " (start " << sn/runs << " / goal " << gn/runs << ")\n";
    std::cout << "tree_mem(KB)   : " << tree_kb << "\n";
    std::cout << "path_mem(KB)   : " << path_kb << "\n";
    std::cout << "SUMMARY model=con runs=" << runs << " success=" << successes
              << " deterministic=" << (deterministic?1:0)
              << " wall_mean=" << wall_mean << " wall_median=" << wall_median
              << " cpu_mean=" << cpu_mean << " cpu_median=" << cpu_median
              << " iterations=" << (it_sum/runs) << " nodes=" << mean_nodes
              << " path_nodes=" << mean_path
              << " nn_checks=" << (nn_sum/runs)
              << " connection_checks=" << (conn_sum/runs)
              << " collision_checks=" << (collision_sum/runs)
              << " tree_mem_kb=" << tree_kb << " path_mem_kb=" << path_kb << "\n";

    if (save_output && last.found) extract_and_save(S,G);
    return 0;
}
