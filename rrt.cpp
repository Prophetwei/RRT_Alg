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
};

std::vector<Node> tree;

bool map[MAP_SIZE][MAP_SIZE];
Node start_node;
Node goal_node;

Node get_random_point() {
    Node point;
    
    // Generate a number between 0 and 99
    int bias = rand() % 100;

    if (bias < 20) { 
        point = goal_node;
    } else {
        // 90% Random Exploration
        point.x = rand() % 100;
        point.y = rand() % 100;
    }
    
    return point;
}


int find_nearest_node(Node rand_point) {
    int index = 0;
    for (int i = 0; i < tree.size(); i++) {
        double distance = MAP_RANGE;
        double temp_dist = sqrt((pow(rand_point.x - tree[i].x, 2)) + (pow(rand_point.y - tree[i].y, 2)));
        if (temp_dist < distance) {
            distance = temp_dist;
            index = i;
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
        // Correct vector: (Target - Start)
        double vec_x = (rand_point.x - nearest_node.x) / dist;
        double vec_y = (rand_point.y - nearest_node.y) / dist;

        new_node.x = nearest_node.x + vec_x * SEARCH_RANGE;
        new_node.y = nearest_node.y + vec_y * SEARCH_RANGE;
    }
    return new_node;
}

bool is_collision(Node nearest_node, Node new_node) {
    // Bresenham's Line Algorithm for collision checking
    int x0 = static_cast<int>(nearest_node.x);
    int y0 = static_cast<int>(nearest_node.y);
    int x1 = static_cast<int>(new_node.x);
    int y1 = static_cast<int>(new_node.y);

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1; 
    int err = dx + dy, e2;

    while (true) {
        if (map[x0][y0]) return true; // Collision with obstacle
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
    
    return false; // No collision
}

bool goal_found() {
    if (tree.empty()) return false;
    Node last_node = tree.back();
    double dist = sqrt(pow(last_node.x - goal_node.x, 2) + pow(last_node.y - goal_node.y, 2));
    
    return dist < 1.0;
}

void rrt_step() {
    // 1. Pick random point
    Node rand_point = get_random_point();
    
    // 2. Find nearest neighbor in the current tree
    int nearest_idx = find_nearest_node(rand_point); 
    
    // 3. Create new_node by "steering" toward rand_point
    Node new_node = steer(tree[nearest_idx], rand_point);
    
    // 4. Check for collision [cite: 176]
    if (!is_collision(tree[nearest_idx], new_node)) {
        new_node.parent_index = nearest_idx;
        tree.push_back(new_node); // Add to tree [cite: 455]
    }
}

void extract_path() {
    std::vector<Node> final_path;
    int current_idx = tree.size() - 1; // Start from the node that reached the goal

    while (current_idx != 0) { // Index 0 is the start_node
        final_path.push_back(tree[current_idx]);
        current_idx = tree[current_idx].parent_index; // Jump to the parent
    }
    final_path.push_back(tree[0]); // Add the start node

    // The path is currently Goal -> Start. 
    // You can print it in reverse to see Start -> Goal.
    std::cout << "Path found! Nodes in path: " << final_path.size() << std::endl;
    for (int i = final_path.size() - 1; i >= 0; i--) {
        std::cout << "(" << final_path[i].x << ", " << final_path[i].y << ") -> ";
    }
    std::cout << "GOAL" << std::endl;
}

int main() {
    // 1. Load Map Data (Mimics loading SRAM in the processor)
    std::ifstream file("map/map.txt"); // Use quotes
    if (!file.is_open()) {
        std::cerr << "Error: Could not find map.txt. Run your python script first!" << std::endl;
        return 1;
    }

    for (int y = 0; y < 100; y++) {
        for (int x = 0; x < 100; x++) {
            int val;
            if (!(file >> val)) break; 
            map[x][y] = (val == 1); // 1 = Obstacle
        }
    }
    file.close();

    // for (int i = 0; i < 100; i++) {
    //     for (int j = 0; j < 100; j++) {
    //         std::cout << map[i][j] << " ";
    //     }        std::cout << std::endl;
    // }

    // 2. Set Start and Goal (Param. Configurator stage)
    start_node.x = 2.0; 
    start_node.y = 2.0; 
    start_node.parent_index = -1; // Root has no parent
    
    goal_node.x = MAP_RANGE - 5.0; 
    goal_node.y = MAP_RANGE - 5.0;

    tree.clear();
    tree.push_back(start_node); 

    // 3. RRT Loop (The Tree Expansion stage)
    std::cout << "Planning path..." << std::endl;
    int iterations = 0;
    const int MAX_ITER = 100000; // Safety exit

    while (!goal_found() && iterations < MAX_ITER) {
        rrt_step();
        std::cout << "New node: " << tree.back().x << ", " << tree.back().y << " | Iteration: " << iterations << std::endl;
        iterations++;
    }

    // 4. Output (Path Tracer stage)
    if (goal_found()) {
        std::cout << "Goal reached in " << iterations << " iterations!" << std::endl;
        extract_path();
    } else {
        std::cout << "Failed to find path within iteration limit." << std::endl;
    }

    return 0;
}