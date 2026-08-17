import matplotlib.pyplot as plt
import numpy as np
import os

def visualize_map(map_index, map_file, path_file, start_nodes, goal_nodes, output_file):
    if not os.path.exists(map_file):
        print(f"Map file missing: {map_file}")
        return

    map_data = np.loadtxt(map_file)
    if map_data.ndim == 1:
        map_data = np.expand_dims(map_data, axis=0)

    def load_tree_data(fname):
        if not os.path.exists(fname):
            return np.array([])
        data = []
        with open(fname, "r") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 5:
                    try:
                        data.append([
                            int(parts[0]),
                            float(parts[1]),
                            float(parts[2]),
                            int(parts[3]),
                            int(parts[4]),
                        ])
                    except ValueError:
                        continue
        return np.array(data)

    def load_path(fname):
        if not os.path.exists(fname):
            return np.array([])
        path_data = np.loadtxt(fname)
        if getattr(path_data, "ndim", 1) == 1:
            path_data = np.expand_dims(path_data, axis=0)
        return path_data

    s_tree_data = load_tree_data(start_nodes)
    g_tree_data = load_tree_data(goal_nodes)
    path = load_path(path_file)

    plt.figure(figsize=(10, 10))
    rows, cols = map_data.shape
    plt.imshow(map_data, cmap='Greys', origin='lower', extent=[0, cols, 0, rows])

    def draw_tree(tree_data, color, label):
        if tree_data.size == 0:
            return

        index_to_row = {int(row[0]): row for row in tree_data}

        for i in range(len(tree_data)):
            parent_idx = int(tree_data[i, 3])
            if parent_idx != -1 and parent_idx in index_to_row:
                parent = index_to_row[parent_idx]
                child = tree_data[i]
                plt.plot([child[1], parent[1]], [child[2], parent[2]],
                         color=color, linewidth=0.5, alpha=0.4)

        invalid = tree_data[tree_data[:, 4] == 0]
        if invalid.size > 0:
            plt.scatter(invalid[:, 1], invalid[:, 2], c='black', s=5, marker='x', alpha=0.5)

        plt.scatter(tree_data[:, 1], tree_data[:, 2], c=color, s=2, alpha=0.6, label=label)

    draw_tree(s_tree_data, 'lime', 'Start Tree')
    draw_tree(g_tree_data, 'cyan', 'Goal Tree')

    if path.size > 0:
        plt.plot(path[:, 0], path[:, 1], color='red', linewidth=2.5, label='Final Path', zorder=10)
        plt.scatter(path[0, 0], path[0, 1], color='green', s=40, label='Start', zorder=11)
        plt.scatter(path[-1, 0], path[-1, 1], color='blue', s=40, label='Goal', zorder=11)

    plt.title(f"Bi-RRT Tree Growth & Path Visualization - Map {map_index}")
    plt.legend(loc='upper right', markerscale=3)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.tight_layout()
    plt.savefig(output_file, dpi=300)
    plt.close()
    print(f"Saved {output_file}")


def visualize_all(num_maps=10):
    for map_index in range(num_maps):
        map_file = f"map/map{map_index}.txt"
        path_file = f"tree/path{map_index}"
        start_nodes = f"tree/start_nodes{map_index}"
        goal_nodes = f"tree/goal_nodes{map_index}"
        output_file = f"result/tree_growth_visualization{map_index}.png"
        visualize_map(map_index, map_file, path_file, start_nodes, goal_nodes, output_file)


if __name__ == "__main__":
    visualize_all(10)