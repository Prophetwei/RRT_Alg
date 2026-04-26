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

const double MAP_RANGE = 1000.0;
const int MAP_SIZE = 1000;
const double SEARCH_RANGE = 5.0;

struct Node {
    double x, y;
    int parent_index;
    bool valid;
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
    Node point;
    if ((rand() % 100) < bias_threshold) {
        return target;
    }
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(0.0, MAP_RANGE - 1.0);
    point.x = dis(gen);
    point.y = dis(gen);
    return point;
}

// Unified Nearest Neighbor Search
int find_nearest_node(Node rand_point, const std::vector<Node>& current_tree) {
    int index = 0;
    double min_dist = 1e9;
    for (int i = 0; i < (int)current_tree.size(); i++) {
        if (!current_tree[i].valid) continue; // Skip invalid nodes
        // Using L1 distance for speed, matching hardware optimization
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
        if (!other_tree[i].valid) continue;
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
        parent_idx = active_tree.size() - 1;

        check_connection(active_tree, other_tree, is_start_tree);
        if (done_flag) return;
    }
}

void extract_path(int path_count, int iteration) {
    if (start_conn_idx == -1 || goal_conn_idx == -1) return;

    std::vector<Node> full_path;

    // 1. Trace from Start Connection back to Start Root
    int curr = start_conn_idx;
    while (curr != -1) {
        full_path.insert(full_path.begin(), start_tree[curr]);
        curr = start_tree[curr].parent_index;
    }

    // 2. Trace from Goal Connection back to Goal Root
    curr = goal_conn_idx;
    while (curr != -1) {
        full_path.push_back(goal_tree[curr]);
        curr = goal_tree[curr].parent_index;
    }

    // 3. Output to Console
    std::cout << "Path found! Total nodes: " << full_path.size() << std::endl;

    // 4. Output to File "path"
    std::ofstream path_file("path/path" + std::to_string(path_count) + ".txt");
    if (path_file.is_open()) {
        path_file << "Iteration: " << iteration << "\n";
        for (const auto& node : full_path) {
            // Writing as "x y" per line for easy parsing by scripts
            path_file << node.x << " " << node.y << "\n";
        }
        path_file.close();
        std::cout << "Path coordinates saved to file: 'path'" << std::endl;
    } else {
        std::cerr << "Error: Could not create path file." << std::endl;
    }
}

void update_tree_validity() {
    // Prune Start Tree
    for (int i = 0; i < (int)start_tree.size(); i++) {
        if (!start_tree[i].valid) continue;

        // Check node and edge to parent
        if (i > 0) {
            if (is_collision(start_tree[start_tree[i].parent_index], start_tree[i])) {
                start_tree[i].valid = false;
            } else if (!start_tree[start_tree[i].parent_index].valid) {
                // Propagation
                start_tree[i].valid = false;
            }
        } else {
            // Root node check
            if (map_memory[(int)start_tree[i].x][(int)start_tree[i].y]) start_tree[i].valid = false;
        }
    }

    // Prune Goal Tree
    for (int i = 0; i < (int)goal_tree.size(); i++) {
        if (!goal_tree[i].valid) continue;

        if (i > 0) {
            if (is_collision(goal_tree[goal_tree[i].parent_index], goal_tree[i])) {
                goal_tree[i].valid = false;
            } else if (!goal_tree[goal_tree[i].parent_index].valid) {
                // Propagation
                goal_tree[i].valid = false;
            }
        } else {
            // Root node check
            if (map_memory[(int)goal_tree[i].x][(int)goal_tree[i].y]) goal_tree[i].valid = false;
        }
    }
    done_flag = false; // Reset done flag to allow continued planning with updated trees
}

int main() {
    // Initialization omitted for brevity - load map.txt as before
    start_node = {2.0, 2.0, -1, true};
    goal_node = {MAP_RANGE - 5, MAP_RANGE - 5, -1, true};
    start_tree.push_back(start_node);
    goal_tree.push_back(goal_node);
    int path_cnt = 0;

    for (int i = 0; i < 10000; i++) {
        if (i % 1000 == 0) {
            int map_index = (i / 1000) % 10; // Loops back to map0 after map9
            std::ifstream file("map/map" + std::to_string(map_index) + ".txt");
            if (!file.is_open()) {
                std::cerr << "Error: Could not find map.txt. Run your python script first!" << std::endl;
                return 1;
            }

            for (int y = 0; y < 100; y++) {
                for (int x = 0; x < 100; x++) {
                    int val;
                    if (!(file >> val)) break; 
                    map_memory[x][y] = (val == 1);
                }
            }
            file.close();
            update_tree_validity(); 
        }
        // Grow Start Tree
        Node rand_s = get_random_point(goal_node, 3);
        extend_branch(find_nearest_node(rand_s, start_tree), rand_s, start_tree, goal_tree, true);
        
        if (done_flag) {
            path_cnt++;
            //extract_path(path_cnt, i);
        }

        // Grow Goal Tree
        Node rand_g = get_random_point(start_node, 5);
        extend_branch(find_nearest_node(rand_g, goal_tree), rand_g, goal_tree, start_tree, false);

        if (done_flag) {
            path_cnt++;
            //extract_path(path_cnt, i);
        }

        std::cout <<"At iteration: " << i << "found path" << std::endl;
    }

    return 0;
}