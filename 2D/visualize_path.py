import matplotlib.pyplot as plt
import numpy as np
import os

def visualize_all(map_file="map/map.txt", path_file="tree/path", start_nodes="tree/start_nodes", goal_nodes="tree/goal_nodes"):
    # 1. Load Map
    if not os.path.exists(map_file):
        print(f"Error: {map_file} missing.")
        return
    map_data = np.loadtxt(map_file)
    if map_data.ndim == 1:
        map_data = np.expand_dims(map_data, axis=0)
    
    # 2. Updated Loader to handle: [X, Y, Parent_Index]
    def load_tree_data(fname):
        if not os.path.exists(fname): 
            print(f"Warning: {fname} not found.")
            return np.array([])
        data = []
        with open(fname, "r") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        # [x, y, parent_idx]
                        data.append([float(parts[0]), float(parts[1]), int(parts[2])])
                    except ValueError:
                        continue
        return np.array(data)

    def load_path(fname):
        if not os.path.exists(fname): return np.array([])
        try:
            return np.loadtxt(fname)
        except:
            return np.array([])

    s_tree_data = load_tree_data(start_nodes)
    g_tree_data = load_tree_data(goal_nodes)
    path = load_path(path_file)

    # 3. Plotting
    plt.figure(figsize=(10, 10))
    rows, cols = map_data.shape
    # Use 'lower' origin to match C++ (0,0) at bottom-left
    plt.imshow(map_data, cmap='Greys', origin='lower', extent=[0, cols, 0, rows])

    # Function to draw tree branches using parent indices
    def draw_tree(tree_data, color, label):
        if tree_data.size == 0: return
        
        # Draw branches (lines from child to parent)
        for i in range(len(tree_data)):
            parent_idx = int(tree_data[i, 2])
            # -1 is the root; we check bounds to ensure the parent exists in the current file
            if parent_idx != -1 and parent_idx < len(tree_data):
                parent = tree_data[parent_idx]
                child = tree_data[i]
                plt.plot([child[0], parent[0]], [child[1], parent[1]], 
                         color=color, linewidth=0.6, alpha=0.4)
        
        # Plot node points
        plt.scatter(tree_data[:, 0], tree_data[:, 1], c=color, s=3, alpha=0.7, label=label)

    # Draw Start Tree (Lime) and Goal Tree (Cyan)
    draw_tree(s_tree_data, 'lime', 'Start Tree')
    draw_tree(g_tree_data, 'cyan', 'Goal Tree')

    # 4. Highlight the Final Path
    if path.size > 0:
        plt.plot(path[:, 0], path[:, 1], color='red', linewidth=3, label='Final Path', zorder=10)
        plt.scatter(path[0, 0], path[0, 1], color='green', s=50, label='Start Node', zorder=11)
        plt.scatter(path[-1, 0], path[-1, 1], color='blue', s=50, label='Goal Node', zorder=11)

    plt.title("RRT Dynamic Growth Visualization")
    plt.legend(loc='upper right', markerscale=2)
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    
    # Save output
    output_name = "rrt_output_result.png"
    plt.savefig(output_name, dpi=300)
    print(f"Visualization saved as {output_name}")
    plt.show()

if __name__ == "__main__":
    visualize_all()