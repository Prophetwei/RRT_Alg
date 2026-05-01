import matplotlib.pyplot as plt
import numpy as np
import os

def visualize_all(map_file="map/map.txt", path_file="tree/path", start_nodes="tree/start_nodes", goal_nodes="tree/goal_nodes"):
    # 1. Load Map
    if not os.path.exists(map_file):
        print("Map file missing.")
        return
    map_data = np.loadtxt(map_file)
    # Ensure map is 2D
    if map_data.ndim == 1:
        map_data = np.expand_dims(map_data, axis=0)
    
    # 2. Load Trees and Path
    def load_coords(fname):
        if not os.path.exists(fname): return np.array([])
        coords = []
        with open(fname, "r") as f:
            for line in f:
                if "Iteration" in line: continue
                parts = line.split()
                # Accept lines with at least 2 columns (ignore extras)
                if len(parts) >= 2:
                    try:
                        coords.append([float(parts[0]), float(parts[1])])
                    except ValueError:
                        continue
        return np.array(coords)

    path = load_coords(path_file)
    s_tree = load_coords(start_nodes)
    g_tree = load_coords(goal_nodes)

    # 3. Plotting
    plt.figure(figsize=(8, 8))

    # Map dimensions and extent (x cols, y rows)
    rows, cols = map_data.shape
    extent = [0, cols, 0, rows]

    # Show the obstacles
    plt.imshow(map_data, cmap='Greys', origin='lower', extent=extent)
    
    # Helper: classify nodes whether they fall into obstacle cells
    def classify_nodes(nodes):
        if nodes.size == 0:
            return np.array([]), np.array([])
        xs = nodes[:, 0].astype(int)
        ys = nodes[:, 1].astype(int)
        # clamp indices
        xs = np.clip(xs, 0, cols - 1)
        ys = np.clip(ys, 0, rows - 1)
        in_obs_mask = map_data[ys, xs] != 0
        in_obs = nodes[in_obs_mask]
        free = nodes[~in_obs_mask]
        return free, in_obs

    # Plot Start Tree (Light Green) and mark nodes inside obstacles
    s_free, s_inobs = classify_nodes(s_tree)
    if s_free.size > 0:
        plt.scatter(s_free[:, 0], s_free[:, 1], c='lime', s=1, alpha=0.3, label='Start Tree Exploration')
    if s_inobs.size > 0:
        plt.scatter(s_inobs[:, 0], s_inobs[:, 1], c='black', s=6, marker='x', label='Start Nodes in Obstacle')

    # Plot Goal Tree (Light Blue) and mark nodes inside obstacles
    g_free, g_inobs = classify_nodes(g_tree)
    if g_free.size > 0:
        plt.scatter(g_free[:, 0], g_free[:, 1], c='cyan', s=1, alpha=0.3, label='Goal Tree Exploration')
    if g_inobs.size > 0:
        plt.scatter(g_inobs[:, 0], g_inobs[:, 1], c='magenta', s=6, marker='x', label='Goal Nodes in Obstacle')
    
    # Highlight the Final Path (Bold Red) and mark any path points inside obstacles
    if path.size > 0:
        plt.plot(path[:, 0], path[:, 1], color='red', linewidth=2, label='Final Path', zorder=10)
        plt.scatter(path[0, 0], path[0, 1], color='green', s=16, label='Start', zorder=11)
        plt.scatter(path[-1, 0], path[-1, 1], color='blue', s=16, label='Goal', zorder=11)
        # path collisions
        p_free, p_inobs = classify_nodes(path)
        if p_inobs.size > 0:
            plt.scatter(p_inobs[:, 0], p_inobs[:, 1], c='red', s=20, marker='X', label='Path Points in Obstacle')

    # Print simple counts for quick debugging
    print(f"Start nodes: {len(s_tree)}  (in obstacle: {len(s_inobs)})")
    print(f"Goal nodes: {len(g_tree)}  (in obstacle: {len(g_inobs)})")
    if path.size > 0:
        print(f"Path points in obstacle: {len(p_inobs)}")

    plt.title("Bi-RRT Exploration & Path Highlight")
    plt.legend(loc='upper right', markerscale=5)
    plt.xlabel("X"); plt.ylabel("Y")
    
    # Save the output for review
    plt.savefig("full_exploration.png", dpi=300)
    plt.show()

if __name__ == "__main__":
    visualize_all()