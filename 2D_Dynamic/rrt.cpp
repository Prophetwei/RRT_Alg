//optimized with dual-tree, branch extension
//add dynamic map
//compile command:
//g++ -std=c++17 rrt_opt.cpp -o rrt_opt

//further optimizations:    1) When the map is updated, only check the nodes that are close to the updated area. If a node is found to be in collision, mark it as invalid and propagate this invalidity to its children. This way we can avoid re-checking the entire tree and only focus on the affected branches.
//                 My idea! 2) When finding the nearest node, starts from the root and its children. Chsen the closest child node as the new sub-tree to search. The nearest node is found when the node is a root.
//                              Which reduced the searching space to  lg(N).
//Hardware optimization: 1) Size of the new node FIFO is equal to the cycles needed by collision check.

//Problem for improving nearest node searching: If record every childeren, the required memoery will also grows exponentially. What if only record the child that closest to the goal?

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <string>
#include <algorithm>

const int MAP_SIZE = 1024;

struct Node {
    int x, y;
    int parent_index;
    bool valid = true;
};

// Global State
std::vector<Node> start_tree, goal_tree, full_path;
bool map_memory[MAP_SIZE][MAP_SIZE];
Node start_node, goal_node;
bool found_path = false;
bool is_start_tree;
int start_conn_idx = -1;
int goal_conn_idx = -1;

// Unified Random Point Generator with Goal Bias
Node get_random_point(const Node& target, int bias_threshold) {
    if ((rand() % 100) < bias_threshold) return target; // Bias towards target
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0, MAP_SIZE - 1);
    return {static_cast<int>(dis(gen)), static_cast<int>(dis(gen)), -1, true};
}

// Unified Nearest Neighbor Search
int find_nearest_node(Node rand_point) {
    int index = 0;
    double min_dist =10e9;
    for (int i = 0; i < (int)start_tree.size(); i++) {
        if (!start_tree[i].valid) continue; // Skip invalid nodes
        double dist = std::abs(rand_point.x - start_tree[i].x) + std::abs(rand_point.y - start_tree[i].y);
        if (dist < min_dist) {
            min_dist = dist;
            index = i;
            is_start_tree = true;
        }
    }
    for (int i = 0; i < (int)goal_tree.size(); i++) {
        if (!goal_tree[i].valid) continue; // Skip invalid nodes
        double dist = std::abs(rand_point.x - goal_tree[i].x) + std::abs(rand_point.y - goal_tree[i].y);
        if (dist < min_dist) {
            min_dist = dist;
            index = i;
            is_start_tree = false;
        }
    }
    return index;
}

// Collision Detection (Bresenham's)
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
        if (!other_tree[i].valid) continue; // Skip invalid nodes

        int dx = std::abs(last_node.x - other_tree[i].x);
        int dy = std::abs(last_node.y - other_tree[i].y);
        if (dx <= 1 && dy <= 1) {
            found_path = true;
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
    new_node.valid = true;
    while (true) {
        if (is_collision(new_node)) break;

        (is_start_tree) ? start_tree.push_back(new_node) : goal_tree.push_back(new_node);
        new_node.x = new_node.x + vector[0];
        new_node.y = new_node.y + vector[1];
        new_node.parent_index = (is_start_tree) ? start_tree.size() - 1 : goal_tree.size() - 1;
        new_node.valid = true;

        check_connection();
        if (found_path) return;
    }
}

void extract_path() {
    if (start_conn_idx == -1 || goal_conn_idx == -1) return;
    full_path.clear();
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

void save_path_snapshot(int map_index) {
    if (full_path.empty()) return;

    std::ofstream path_file("tree/path" + std::to_string(map_index));
    for (const auto& n : full_path) {
        path_file << n.x << " " << n.y << "\n";
    }
    path_file.close();
    std::cout << "Path snapshot saved to tree/path" << map_index << std::endl;
}

void update_tree_validity() {
    // Check nodes against the new map
    for (auto& tree : {&start_tree, &goal_tree}) {
        for (int i = 0; i < tree->size(); i++) {
            Node& node = (*tree)[i];
            if (!node.valid) continue;

            // 1. Check if node itself is in collision
            if (map_memory[node.x][node.y]) {
                node.valid = false;
            } 
            // 2. Check if parent is invalid (Propagation)
            else if (node.parent_index != -1 && !(*tree)[node.parent_index].valid) {
                node.valid = false;
            }
            // 3. Optional: Re-check the EDGE to the parent for new obstacles
        }
    }
}

void check_path() {
    for (const auto& node : full_path) {
        if (map_memory[node.x][node.y]) {
            found_path = false;
            full_path.clear();
            return;
        }
    }
}

void save_all_nodes(int map_index) {
    std::ofstream s_file("tree/start_nodes" + std::to_string(map_index));
    for (int i = 0; i < (int)start_tree.size(); i++) {
        const auto& node = start_tree[i];
        if (node.valid) {
            s_file << i << " " << node.x << " " << node.y << " "
                   << node.parent_index << " " << node.valid << "\n";
        }
    }
    s_file.close();

    std::ofstream g_file("tree/goal_nodes" + std::to_string(map_index));
    for (int i = 0; i < (int)goal_tree.size(); i++) {
        const auto& node = goal_tree[i];
        if (node.valid) {
            g_file << i << " " << node.x << " " << node.y << " "
                   << node.parent_index << " " << node.valid << "\n";
        }
    }
    g_file.close();
    std::cout << "All tree nodes saved to tree/start_nodes" << map_index
              << " and tree/goal_nodes" << map_index << std::endl;
}

int main() {
    // Initialization omitted for brevity - load map.txt as before
    start_node = {0, 0, -1, true};
    goal_node = {MAP_SIZE - 1, MAP_SIZE - 1, -1, true};
    start_tree.push_back(start_node);
    goal_tree.push_back(goal_node);

    for (int i = 0; i < 100000; i++) {
        if (i % 10000 == 0) {
            std::cout << "Update map" << std::endl;
            int map_index = (i / 10000) % 10; // Loops back to map0 after map9
            std::ifstream file("map/map" + std::to_string(map_index) + ".txt");
            if (!file.is_open()) {
                std::cerr << "Error: Could not find map.txt. Run your python script first!" << std::endl;
                return 1;
            }

            for (int y = 0; y < MAP_SIZE; y++) {
                for (int x = 0; x < MAP_SIZE; x++) {
                    map_memory[x][y] = false;
                }
            }

            for (int y = 0; y < MAP_SIZE; y++) {
                for (int x = 0; x < MAP_SIZE; x++) {
                    int val;
                    if (!(file >> val)) break; 
                    map_memory[x][y] = (val == 1);
                }
            }
            file.close();

            update_tree_validity(); 
            check_path();

            if (found_path) {
                extract_path();
                save_all_nodes(map_index);
                save_path_snapshot(map_index);
            }

            if (!found_path) {
                std::cout << "Path blocked or not found. Resuming growth." << std::endl;
            } else {
                std::cout << "Existing path still valid. Staying idle." << std::endl;
            }
        }

        // Grow Tree
        if (!found_path) {
            Node bias_target = (i % 2 == 0) ? goal_node : start_node;
            Node rand_s = get_random_point(bias_target, 5);
            int parent_idx = find_nearest_node(rand_s);
            extend_branch(parent_idx, rand_s);

            if (found_path) {
                std::cout << "New path found! Stopping growth." << std::endl;
                extract_path();
                int map_index = (i / 10000) % 10;
                save_all_nodes(map_index);
                save_path_snapshot(map_index);
            }
        }
    }

    return 0;
}