import matplotlib.pyplot as plt
import numpy as np
import os

def visualize_all(map_file="map/map.txt", path_file="tree/path", start_nodes="tree/start_nodes", goal_nodes="tree/goal_nodes"):
    # 1. Load Map
    if not os.path.exists(map_file):
        print("Map file missing.")
        return
    map_data = np.loadtxt(map_file)
    
    # 2. Load Trees and Path
    def load_coords(fname):
        if not os.path.exists(fname): return np.array([])
        coords = []
        with open(fname, "r") as f:
            for line in f:
                if "Iteration" in line: continue
                parts = line.split()
                if len(parts) == 2: coords.append([float(parts[0]), float(parts[1])])
        return np.array(coords)

    path = load_coords(path_file)
    s_tree = load_coords(start_nodes)
    g_tree = load_coords(goal_nodes)

    # 3. Plotting
    plt.figure(figsize=(8, 8))
    
    # Show the obstacles
    plt.imshow(map_data, cmap='Greys', origin='lower', extent=[0, 1000, 0, 1000])
    
    # Plot Start Tree (Light Green)
    if s_tree.size > 0:
        plt.scatter(s_tree[:, 0], s_tree[:, 1], c='lime', s=1, alpha=0.3, label='Start Tree Exploration')
    
    # Plot Goal Tree (Light Blue)
    if g_tree.size > 0:
        plt.scatter(g_tree[:, 0], g_tree[:, 1], c='cyan', s=1, alpha=0.3, label='Goal Tree Exploration')
    
    # Highlight the Final Path (Bold Red)
    if path.size > 0:
        plt.plot(path[:, 0], path[:, 1], color='red', linewidth=2, label='Final Path', zorder=10)
        plt.scatter(path[0, 0], path[0, 1], color='green', s=1, label='Start', zorder=11)
        plt.scatter(path[-1, 0], path[-1, 1], color='blue', s=1, label='Goal', zorder=11)

    plt.title("Bi-RRT Exploration & Path Highlight")
    plt.legend(loc='upper right', markerscale=5)
    plt.xlabel("X"); plt.ylabel("Y")
    
    # Save the output for review
    plt.savefig("full_exploration.png", dpi=300)
    plt.show()

if __name__ == "__main__":
    visualize_all()