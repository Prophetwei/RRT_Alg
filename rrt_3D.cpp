//optimized with dual-tree, branch extension
//compile command: g++ -std=c++17 rrt_opt.cpp -o rrt_opt

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <string>
#include <algorithm>

const double MAP_RANGE = 500.0;
const int MAP_SIZE = 500;
const double SEARCH_RANGE = 3.0;

struct Node {
    double x, y, z;
    int parent_index;
};

// Global State
std::vector<Node> start_tree, goal_tree;
std::vector<std::vector<std::vector<bool>>> map_memory;Node start_node, goal_node;
bool done_flag = false;
int start_conn_idx = -1;
int goal_conn_idx = -1;

void init_map() {
    map_memory.resize(MAP_SIZE, std::vector<std::vector<bool>>(MAP_SIZE, std::vector<bool>(MAP_SIZE, false)));
}

// Unified Random Point Generator with Goal Bias
Node get_random_point(const Node& target, int bias_threshold) {
    if ((rand() % 100) < bias_threshold) return target;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, MAP_RANGE - 1.0);
    return {dis(gen), dis(gen), dis(gen), -1};
}

// Unified Nearest Neighbor Search
int find_nearest_node(const Node& p, const std::vector<Node>& tree) {
    int idx = 0; double min_d = 1e9;
    for (int i = 0; i < tree.size(); ++i) {
        double d = std::sqrt(std::pow(p.x-tree[i].x,2)+std::pow(p.y-tree[i].y,2)+std::pow(p.z-tree[i].z,2));
        if (d < min_d) { min_d = d; idx = i; }
    }
    return idx;
}

// Collision Detection (Bresenham's)
bool is_collision(Node n1, Node n2) {
    int x0 = std::clamp((int)n1.x, 0, MAP_SIZE-1);
    int y0 = std::clamp((int)n1.y, 0, MAP_SIZE-1);
    int z0 = std::clamp((int)n1.z, 0, MAP_SIZE-1);
    int x1 = std::clamp((int)n2.x, 0, MAP_SIZE-1);
    int y1 = std::clamp((int)n2.y, 0, MAP_SIZE-1);
    int z1 = std::clamp((int)n2.z, 0, MAP_SIZE-1);

    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int dz = std::abs(z1 - z0), sz = z0 < z1 ? 1 : -1;

    int dm = std::max({dx, dy, dz});
    int i = dm;
    int x_err = dm/2, y_err = dm/2, z_err = dm/2;

    while (i--) {
        if (map_memory[x0][y0][z0]) return true;
        x_err -= dx; if (x_err < 0) { x_err += dm; x0 += sx; }
        y_err -= dy; if (y_err < 0) { y_err += dm; y0 += sy; }
        z_err -= dz; if (z_err < 0) { z_err += dm; z0 += sz; }
    }
    return false;
}

// Connection Checker for Dual-Tree
void check_connection(const std::vector<Node>& active_tree, const std::vector<Node>& other_tree, bool is_start_active) {
    const Node& last_node = active_tree.back();
    for (int i = 0; i < (int)other_tree.size(); i++) {
        double dist = std::sqrt(std::pow(last_node.x - other_tree[i].x, 2) + std::pow(last_node.y - other_tree[i].y, 2) + std::pow(last_node.z - other_tree[i].z, 2));
        if (dist < SEARCH_RANGE && !is_collision(last_node, other_tree[i])) {
            done_flag = true;
            if (is_start_active) {
                start_conn_idx = active_tree.size() - 1;
                goal_conn_idx = i;
            } else {
                goal_conn_idx = active_tree.size() - 1;
                start_conn_idx = i;
            }
            return;
        }
    }
}

// Unified Branch Extension
void extend_branch(int parent_idx, Node target, std::vector<Node>& active_tree, const std::vector<Node>& other_tree, bool is_start_tree) {
    while (true) {
        Node nearest = active_tree[parent_idx];
        double dist = std::sqrt(std::pow(target.x - nearest.x, 2) + std::pow(target.y - nearest.y, 2) + std::pow(target.z - nearest.z, 2));
        if (dist < 0.1) break;

        Node new_node;
        double step = std::min(SEARCH_RANGE, dist);
        new_node.x = nearest.x + (target.x - nearest.x) * step / dist;
        new_node.y = nearest.y + (target.y - nearest.y) * step / dist;
        new_node.z = nearest.z + (target.z - nearest.z) * step / dist;

        if (is_collision(nearest, new_node)) break;

        new_node.parent_index = parent_idx;
        active_tree.push_back(new_node);
        parent_idx = (int)active_tree.size() - 1;

        check_connection(active_tree, other_tree, is_start_tree);
        if (done_flag) return;
    }
}

void extract_path() {
    std::vector<Node> path;
    int c = start_conn_idx;
    while(c != -1) { path.insert(path.begin(), start_tree[c]); c = start_tree[c].parent_index; }
    c = goal_conn_idx;
    while(c != -1) { path.push_back(goal_tree[c]); c = goal_tree[c].parent_index; }

    std::ofstream f("tree/path");
    for (auto& n : path) f << n.x << " " << n.y << " " << n.z << "\n";
    f.close();
    std::cout << "Path Saved. Total Nodes: " << path.size() << "\n";
}

void save_all_nodes() {
    std::ofstream s_file("tree/start_nodes");
    for (const auto& node : start_tree) {
        s_file << node.x << " " << node.y << " " << node.z << "\n";
    }
    s_file.close();

    std::ofstream g_file("tree/goal_nodes");
    for (const auto& node : goal_tree) {
        g_file << node.x << " " << node.y << " " << node.z << "\n";
    }
    g_file.close();
    std::cout << "All tree nodes saved to 'start_nodes' and 'goal_nodes'" << std::endl;
}

void print_memory_stats() {
    size_t map_mem = sizeof(map_memory); // Static grid size
    size_t node_size = sizeof(Node);
    size_t tree_mem = (start_tree.size() + goal_tree.size()) * node_size;

    std::cout << "--- Memory Stats ---" << std::endl;
    std::cout << "Map Grid: " << map_mem / 1024.0 << " KB" << std::endl;
    std::cout << "Trees (Nodes): " << tree_mem / 1024.0 << " KB" << std::endl;
    std::cout << "Total Estimated: " << (map_mem + tree_mem) / 1024.0 << " KB" << std::endl;
}

int main() {
    srand(time(0));

    init_map();
    
    // 1. Load the map
    std::ifstream file("map/map_3D.txt");
    if (!file.is_open()) {
        std::cerr << "Error: map_3D.txt not found!" << std::endl;
        return 1;
    }
    int x, y, z;
    while (file >> x >> y >> z) {
        if (x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE && z >= 0 && z < MAP_SIZE) {
            map_memory[x][y][z] = true; // Mark as obstacle
        }
    }
    file.close();

    // 2. Initialize Trees
    start_node = {2.0, 2.0, 2.0, -1};
    goal_node = {MAP_RANGE - 5.0, MAP_RANGE - 5.0, MAP_RANGE - 5.0, -1};
    start_tree.push_back(start_node);
    goal_tree.push_back(goal_node);

    // 3. Main Loop
    std::cout << "Planning..." << std::endl;
    for (int i = 0; i < 10000 && !done_flag; i++) {
        Node rand_s = get_random_point(goal_node, 3);
        extend_branch(find_nearest_node(rand_s, start_tree), rand_s, start_tree, goal_tree, true);
        if (done_flag) break;

        Node rand_g = get_random_point(start_node, 5);
        extend_branch(find_nearest_node(rand_g, goal_tree), rand_g, goal_tree, start_tree, false);
    }

    if (done_flag) {
        extract_path();
        save_all_nodes();
        print_memory_stats();
    }
    else std::cout << "Failed to find path." << std::endl;

    return 0;
}