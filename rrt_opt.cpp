//optimized with dual-tree, branch extension
//compile command:
//g++ -std=c++17 rrt_opt.cpp -o rrt_opt

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <string>
#include <algorithm>

const double MAP_RANGE = 1000.0;
const int MAP_SIZE = 1000;
const double SEARCH_RANGE = 3.0;

struct Node {
    double x, y;
    int parent_index;
};

// Global State
std::vector<Node> start_tree, goal_tree;
bool map_memory[MAP_SIZE][MAP_SIZE];
Node start_node, goal_node;
bool done_flag = false;
int start_conn_idx = -1;
int goal_conn_idx = -1;

// Unified Random Point Generator with Goal Bias
Node get_random_point(const Node& target, int bias_threshold) {
    if ((rand() % 100) < bias_threshold) return target;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, MAP_RANGE - 1.0);
    return {dis(gen), dis(gen), -1};
}

// Unified Nearest Neighbor Search
int find_nearest_node(Node rand_point, const std::vector<Node>& current_tree) {
    int index = 0;
    double min_dist = 1e9;
    for (int i = 0; i < (int)current_tree.size(); i++) {
        double dist = std::abs(rand_point.x - current_tree[i].x) + std::abs(rand_point.y - current_tree[i].y);
        if (dist < min_dist) {
            min_dist = dist;
            index = i;
        }
    }
    return index;
}

// Collision Detection (Bresenham's)
bool is_collision(Node n1, Node n2) {
    int x0 = std::clamp(static_cast<int>(n1.x), 0, MAP_SIZE - 1);
    int y0 = std::clamp(static_cast<int>(n1.y), 0, MAP_SIZE - 1);
    int x1 = std::clamp(static_cast<int>(n2.x), 0, MAP_SIZE - 1);
    int y1 = std::clamp(static_cast<int>(n2.y), 0, MAP_SIZE - 1);

    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (map_memory[x0][y0]) return true;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    return false;
}

// Connection Checker for Dual-Tree
void check_connection(const std::vector<Node>& active_tree, const std::vector<Node>& other_tree, bool is_start_active) {
    const Node& last_node = active_tree.back();
    for (int i = 0; i < (int)other_tree.size(); i++) {
        double dist = std::sqrt(std::pow(last_node.x - other_tree[i].x, 2) + std::pow(last_node.y - other_tree[i].y, 2));
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
        double dist = std::sqrt(std::pow(target.x - nearest.x, 2) + std::pow(target.y - nearest.y, 2));
        if (dist < 0.1) break;

        Node new_node;
        double step = std::min(SEARCH_RANGE, dist);
        new_node.x = nearest.x + (target.x - nearest.x) * step / dist;
        new_node.y = nearest.y + (target.y - nearest.y) * step / dist;

        if (is_collision(nearest, new_node)) break;

        new_node.parent_index = parent_idx;
        active_tree.push_back(new_node);
        parent_idx = (int)active_tree.size() - 1;

        check_connection(active_tree, other_tree, is_start_tree);
        if (done_flag) return;
    }
}

void extract_path() {
    if (start_conn_idx == -1 || goal_conn_idx == -1) return;
    std::vector<Node> full_path;
    int curr = start_conn_idx;
    while (curr != -1) {
        full_path.insert(full_path.begin(), start_tree[curr]);
        curr = start_tree[curr].parent_index;
    }
    curr = goal_conn_idx;
    while (curr != -1) {
        full_path.push_back(goal_tree[curr]);
        curr = goal_tree[curr].parent_index;
    }
    std::cout << "Path found! Nodes: " << full_path.size() << std::endl;
    std::ofstream path_file("tree/path");
    for (const auto& n : full_path) path_file << n.x << " " << n.y << "\n";
    path_file.close();
}

void save_all_nodes() {
    std::ofstream s_file("tree/start_nodes");
    for (const auto& node : start_tree) {
        s_file << node.x << " " << node.y << "\n";
    }
    s_file.close();

    std::ofstream g_file("tree/goal_nodes");
    for (const auto& node : goal_tree) {
        g_file << node.x << " " << node.y << "\n";
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
    
    // 1. Load the map
    std::ifstream file("map/map.txt");
    if (!file.is_open()) {
        std::cerr << "Error: map.txt not found!" << std::endl;
        return 1;
    }
    for (int y = 0; y < MAP_SIZE; y++) {
        for (int x = 0; x < MAP_SIZE; x++) {
            int val;
            file >> val;
            map_memory[x][y] = (val == 1);
        }
    }
    file.close();

    // 2. Initialize Trees
    start_node = {2.0, 2.0, -1};
    goal_node = {MAP_RANGE - 5.0, MAP_RANGE - 5.0, -1};
    start_tree.push_back(start_node);
    goal_tree.push_back(goal_node);

    // 3. Main Loop
    std::cout << "Planning..." << std::endl;
    for (int i = 0; i < 10000 && !done_flag; i++) {
        Node rand_s = get_random_point(goal_node, 5);
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