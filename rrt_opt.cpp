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

const int MAP_RANGE = 1024;
const int MAP_SIZE = 1024;
const int SEARCH_RANGE = 3;

struct Node {
    int x, y;
    int parent_index;
};

// Global State
std::vector<Node> start_tree, goal_tree;
bool map_memory[MAP_SIZE][MAP_SIZE];
Node start_node, goal_node;
bool done_flag = false;
bool is_start_tree;
int start_conn_idx = -1;
int goal_conn_idx = -1;

// Unified Random Point Generator with Goal Bias
Node get_random_point(const Node& target, int bias_threshold) {
    if ((rand() % 100) < bias_threshold) return {512, 512, -1};
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0, MAP_RANGE - 1);
    return {dis(gen), dis(gen), -1};
}

// Unified Nearest Neighbor Search
int find_nearest_node(Node rand_point) {
    int index = 0;
    double min_dist =10e9;
    for (int i = 0; i < (int)start_tree.size(); i++) {
        double dist = std::abs(rand_point.x - start_tree[i].x) + std::abs(rand_point.y - start_tree[i].y);
        if (dist < min_dist) {
            min_dist = dist;
            index = i;
            is_start_tree = true;
        }
    }
    for (int i = 0; i < (int)goal_tree.size(); i++) {
        double dist = std::abs(rand_point.x - goal_tree[i].x) + std::abs(rand_point.y - goal_tree[i].y);
        if (dist < min_dist) {
            min_dist = dist;
            index = i;
            is_start_tree = false;
        }
    }
    return index;
}

// Collision Detection
bool is_collision(Node node) {
    if (node.x < 0 || node.x >= MAP_SIZE || node.y < 0 || node.y >= MAP_SIZE || map_memory[node.x][node.y]) return true;
    return false;
}

// Connection Checker for Dual-Tree
void check_connection() {
    const std::vector<Node> active_tree = (is_start_tree) ? start_tree : goal_tree;
    const std::vector<Node> other_tree = (is_start_tree) ? goal_tree : start_tree;
    const Node& last_node = active_tree.back();
    for (int i = 0; i < (int)other_tree.size(); i++) {
        if (last_node.x == other_tree[i].x && last_node.y == other_tree[i].y) {
            done_flag = true;
            if (is_start_tree) {
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
void extend_branch(int parent_idx, Node sample_node) {
    Node nearest_node = (is_start_tree) ? start_tree[parent_idx] : goal_tree[parent_idx];
    int vector[2];
    int dx = sample_node.x - nearest_node.x;
    int dy = sample_node.y - nearest_node.y;

    if (dx > 0) {
        vector[0] = (2 * dx > dy) ? 1 : 0;
    } else {
        vector[0] = (2 * dx < dy) ? -1 : 0;
    }
    if (dy > 0) {
        vector[1] = (dx > 2 * dy) ? 0 : 1;
    } else {
        vector[1] = (dx < 2 * dy) ? 0 : -1;
    }
    if (vector[0] == 0 && vector[1] == 0) return;

    Node new_node;
    new_node.x = nearest_node.x + vector[0];
    new_node.y = nearest_node.y + vector[1];
    new_node.parent_index = parent_idx;
    for (int i = 0; i < 4; i++) {
        if (is_collision(new_node)) break;

        (is_start_tree) ? start_tree.push_back(new_node) : goal_tree.push_back(new_node);
        new_node.x = new_node.x + vector[0];
        new_node.y = new_node.y + vector[1];
        new_node.parent_index = (is_start_tree) ? start_tree.size() - 1 : goal_tree.size() - 1;

        check_connection();
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
    start_node = {2, 2, -1};
    goal_node = {MAP_RANGE - 5, MAP_RANGE - 5, -1};
    start_tree.push_back(start_node);
    goal_tree.push_back(goal_node);

    // 3. Main Loop
    std::cout << "Planning..." << std::endl;
    for (int i = 0; i < 10000 && !done_flag; i++) {
        Node rand_s = get_random_point(goal_node, 3);
        extend_branch(find_nearest_node(rand_s), rand_s);
        if (done_flag) break;
    }

    if (done_flag) {
        print_memory_stats();
    }
    else std::cout << "Failed to find path." << std::endl;

    extract_path();
    save_all_nodes();

    return 0;
}