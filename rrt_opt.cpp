//static optimized with dual-tree, branch extension
//compile command:
//g++ -std=c++17 rrt_opt.cpp -o rrt_opt

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <algorithm>

constexpr int MAP_SIZE = 1000;
constexpr int MAP_RANGE = 1024;

struct Node {
    int x, y;
    int parent = -1;
};

class BiRRT {
public:
    BiRRT() : gen(rd()), dist(0, MAP_RANGE - 1) {}

    void run(const Node& start_node, const Node& goal_node, int bias = 5) {
        if (!loadMap("map/map.txt")) return;

        initTrees();

        std::cout << "Planning...\n";
        for (int i = 0; i < 10000 && !connected; ++i) {
            grow(goal_node, bias);
            if (connected) break;
        }

        if (connected) {
            extractPath();
            saveTrees();
            printMemory();
        } else {
            std::cout << "Failed to find path.\n";
        }
    }

private:
    bool occupancy_grid[MAP_SIZE][MAP_SIZE]{};

    std::vector<Node> start_tree, goal_tree;

    bool connected = false;
    int start_idx = -1, goal_idx = -1;
    bool start_active = true;

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> dist;

    // ---------------- Core ----------------

    void grow(const Node& goal_node, int bias) {
        Node rnd = sample(goal_node, bias);
        int nearest = nearestIndex(rnd);
        extend(nearest, rnd, active, other);
    }

    Node sample(const Node& goal_node, int bias) {
        if ((rand() % 100) < bias) return goal_node;
        return {dist(gen), dist(gen)};
    }

    int nearestIndex(const Node& p) {
        int best = 0;
        int min_dist = 1e9;

        for (int i = 0; i < (int)start_tree.size(); ++i) {
            int d = std::abs(p.x - start_tree[i].x) + std::abs(p.y - start_tree[i].y);
            if (d < min_dist) {
                min_dist = d;
                best = i;
                start_active = true;
            }
        }
        for (int i = 0; i < (int)goal_tree.size(); ++i) {
            int d = std::abs(p.x - goal_tree[i].x) + std::abs(p.y - goal_tree[i].y);
            if (d < min_dist) {
                min_dist = d;
                best = i;
                start_active = false;
            }
        }
        return best;
    }

    void extend(int parent, Node target) {
        std::vector<Node>& tree = start_active ? start_tree : goal_tree;

        const Node& base = tree[parent];
        int dx = target.x - base.x;
        int dy = target.y - base.y;

        std::pair<int, int> vec;

        bool bound_px, bound_nx, bound_py, bound_ny;
        bound_px = (dy > 2 * dx);
        bound_nx = (dy < -2 * dx);
        bound_py = (dx > 2 * dy);
        bound_ny = (dx < -2 * dy);

        vec = (bound_px || bound_nx) ? std::make_pair(0, (dy > 0) ? 1 : -1) :
              (bound_py || bound_ny) ? std::make_pair((dx > 0) ? 1 : -1, 0) :
              std::make_pair((dx > 0) ? 1 : -1, (dy > 0) ? 1 : -1);

        Node next{
            base.x + vec.first,
            base.y + vec.second,
            parent
        };
        if (collision(next)) return;
        tree.push_back(next);
        parent = tree.size() - 1;

        for (int i = 0; i < 4; i++) {
            next = {next.x + vec.first, next.y + vec.second, parent};

            if (collision(next)) return;

            tree.push_back(next);
            parent = tree.size() - 1;

            if (tryConnect(tree, other)) return;
        }
    }

    bool tryConnect(const std::vector<Node>& a,
                    const std::vector<Node>& b) {

        const Node& last = a.back();

        for (int i = 0; i < (int)b.size(); ++i) {
            double d = std::hypot(last.x - b[i].x, last.y - b[i].y);

            if (d < STEP_SIZE && !collision(last, b[i])) {
                connected = true;
                start_idx = (&a == &start_tree) ? a.size() - 1 : i;
                goal_idx  = (&a == &start_tree) ? i : a.size() - 1;
                return true;
            }
        }
        return false;
    }

    // ---------------- Collision ----------------

    bool collision(Node node) {
        int x0 = std::clamp((int)a.x, 0, MAP_SIZE - 1);
        int y0 = std::clamp((int)a.y, 0, MAP_SIZE - 1);

        if (occupancy_grid[x0][y0]) return true;
        return false;
    }

    // ---------------- I/O ----------------

    bool loadMap(const std::string& path) {
        std::ifstream file(path);
        if (!file) {
            std::cerr << "Error: map.txt not found\n";
            return false;
        }

        for (int y = 0; y < MAP_SIZE; ++y)
            for (int x = 0; x < MAP_SIZE; ++x) {
                int v;
                file >> v;
                occupancy_grid[x][y] = (v == 1);
            }

        return true;
    }

    void initTrees() {
        start_tree = {start_node};
        goal_tree  = {goal_node};
    }

    void extractPath() {
        std::vector<Node> path;

        for (int i = start_idx; i != -1; i = start_tree[i].parent)
            path.insert(path.begin(), start_tree[i]);

        for (int i = goal_idx; i != -1; i = goal_tree[i].parent)
            path.push_back(goal_tree[i]);

        std::cout << "Path found! Nodes: " << path.size() << "\n";

        std::ofstream f("tree/path");
        for (auto& n : path)
            f << n.x << " " << n.y << "\n";
    }

    void saveTrees() {
        std::ofstream s("tree/start_nodes"), g("tree/goal_nodes");

        for (auto& n : start_tree) s << n.x << " " << n.y << "\n";
        for (auto& n : goal_tree)  g << n.x << " " << n.y << "\n";
    }

    void printMemory() {
        size_t map_mem = sizeof(occupancy_grid);
        size_t node_mem = (start_tree.size() + goal_tree.size()) * sizeof(Node);

        std::cout << "--- Memory ---\n";
        std::cout << "Map: " << map_mem / 1024.0 << " KB\n";
        std::cout << "Nodes: " << node_mem / 1024.0 << " KB\n";
    }
};

int main() {
    srand(time(nullptr));  // legacy (can be removed if you fully switch to mt19937)
    BiRRT planner;
    planner.run();
}