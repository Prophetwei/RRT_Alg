#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <string>

const double MAP_RANGE = 100.0;
const int MAP_SIZE = 100;
const double SEARCH_RANGE = 5.0;

struct Node {
    double x, y;
    int parent_index;
    bool valid;
};

std::vector<Node> tree;

bool map[MAP_SIZE][MAP_SIZE];
Node start_node;
Node goal_node;

Node get_random_point() {
    Node point;
    
    int bias = rand() % 100;

    if (bias < 25) {
        point = goal_node;
    } else {
        point.x = rand() % 100;
        point.y = rand() % 100;
    }
    
    return point;
}

bool is_collision(Node nearest_node, Node new_node) {
    int x0 = static_cast<int>(nearest_node.x);
    int y0 = static_cast<int>(nearest_node.y);
    int x1 = static_cast<int>(new_node.x);
    int y1 = static_cast<int>(new_node.y);

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
    int err = dx + dy, e2;

    while (true) {
        if (map[x0][y0]) return true; 
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    
    return false; 
}

int find_nearest_node(Node rand_point) {
    int index = 0;
    for (int i = 0; i < tree.size(); i++) {
        if (tree[i].valid) {
            double distance = MAP_RANGE;
            double temp_dist = sqrt((pow(rand_point.x - tree[i].x, 2)) + (pow(rand_point.y - tree[i].y, 2)));
            if (temp_dist < distance) {
                distance = temp_dist;
                index = i;
            }
        }
    }
    return index;
}

Node steer(Node nearest_node, Node rand_point) {
    Node new_node;
    double dist = sqrt(pow(rand_point.x - nearest_node.x, 2) + pow(rand_point.y - nearest_node.y, 2));

    if (dist <= SEARCH_RANGE) {
        return rand_point; 
    } else {
        double vec_x = (rand_point.x - nearest_node.x) / dist;
        double vec_y = (rand_point.y - nearest_node.y) / dist;

        new_node.x = nearest_node.x + vec_x * SEARCH_RANGE;
        new_node.y = nearest_node.y + vec_y * SEARCH_RANGE;
    }
    return new_node;
}

bool goal_found() {
    if (tree.empty() || !tree.back().valid) return false;
    Node last_node = tree.back();
    double dist = sqrt(pow(last_node.x - goal_node.x, 2) + pow(last_node.y - goal_node.y, 2));
    
    return dist < 1.0;
}

void rrt_step() {
    Node rand_point = get_random_point();
    
    int nearest_idx = find_nearest_node(rand_point); 
    
    Node new_node = steer(tree[nearest_idx], rand_point);
    
    if (!is_collision(tree[nearest_idx], new_node)) {
        new_node.parent_index = nearest_idx;
        new_node.valid = true;
        tree.push_back(new_node); 
    }
}

void update_tree_validity() {
    // We start from the beginning of the tree
    for (int i = 0; i < tree.size(); i++) {
        // 1. If it's already invalid, keep it invalid
        if (!tree[i].valid) continue;

        // 2. Check if the node itself or the path from its parent is now blocked
        if (i > 0) { // Start node (0) has no parent
            if (is_collision(tree[tree[i].parent_index], tree[i])) {
                tree[i].valid = false;
            }
        } else {
            // Special check for the start node itself
            if (map[(int)tree[i].x][(int)tree[i].y]) tree[i].valid = false;
        }
        
        // 3. Propagation: If parent is invalid, child is invalid
        if (i > 0 && !tree[tree[i].parent_index].valid) {
            tree[i].valid = false;
        }
    }
}

void extract_path() {
    std::vector<Node> final_path;
    int current_idx = tree.size() - 1; 

    while (current_idx != 0) { 
        final_path.push_back(tree[current_idx]);
        current_idx = tree[current_idx].parent_index; 
    }
    final_path.push_back(tree[0]); 

    std::cout << "Path found! Nodes in path: " << final_path.size() << std::endl;
    for (int i = final_path.size() - 1; i >= 0; i--) {
        std::cout << "(" << final_path[i].x << ", " << final_path[i].y << ") -> ";
    }
    std::cout << "GOAL" << std::endl;
}

int main() {
    start_node.x = 2.0; 
    start_node.y = 2.0; 
    start_node.parent_index = -1;
    start_node.valid = true;
    
    goal_node.x = MAP_RANGE - 5.0; 
    goal_node.y = MAP_RANGE - 5.0;

    tree.clear();
    tree.push_back(start_node); 

    std::cout << "Planning path..." << std::endl;
    int iterations = 0;
    const int MAX_ITER = 100000;

    while (iterations < MAX_ITER) {
        if (iterations % 1000 == 0) {
            int map_index = (iterations / 1000) % 10; // Loops back to map0 after map9
            std::ifstream file("map/map" + std::to_string(map_index) + ".txt");
            if (!file.is_open()) {
                std::cerr << "Error: Could not find map.txt. Run your python script first!" << std::endl;
                return 1;
            }

            for (int y = 0; y < 100; y++) {
                for (int x = 0; x < 100; x++) {
                    int val;
                    if (!(file >> val)) break; 
                    map[x][y] = (val == 1);
                }
            }
            file.close();
            update_tree_validity();
        }

        rrt_step();
        //std::cout << "New node: " << tree.back().x << ", " << tree.back().y << " | Iteration: " << iterations << std::endl;
        iterations++;

        if (goal_found()) {
            std::cout << "Goal reached in " << iterations << " iterations!" << std::endl;
            extract_path();
        }
    }

    if (goal_found()) {
        std::cout << "Goal reached in " << iterations << " iterations!" << std::endl;
        extract_path();
    } else {
        std::cout << "Failed to find path within iteration limit." << std::endl;
    }

    return 0;
}