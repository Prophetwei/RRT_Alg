import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def load_coords(filename):
    """Load coordinates from file (x y z format)"""
    try:
        data = np.loadtxt(filename)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        return data
    except FileNotFoundError:
        print(f"Warning: {filename} not found")
        return np.array([])

def visualize_3d_rrt():
    """Visualize 3D RRT with map, trees, and path"""
    
    # Load data
    print("Loading map obstacles...")
    obstacles = load_coords("map_3D.txt")
    
    print("Loading tree nodes...")
    start_nodes = load_coords("tree/start_nodes_3D")
    goal_nodes = load_coords("tree/goal_nodes_3D")
    
    print("Loading path...")
    path = load_coords("tree/path_3D")
    
    # Create figure
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot obstacles (sparse - sample every nth point to avoid overcrowding)
    if len(obstacles) > 0:
        sample_rate = max(1, len(obstacles) // 5000)  # Limit to ~5000 points
        obs_sample = obstacles[::sample_rate]
        ax.scatter(obs_sample[:, 0], obs_sample[:, 1], obs_sample[:, 2], 
                  c='red', s=1, alpha=0.3, label='Obstacles')
    
    # Plot start tree
    if len(start_nodes) > 0:
        ax.scatter(start_nodes[:, 0], start_nodes[:, 1], start_nodes[:, 2], 
                  c='blue', s=10, alpha=0.6, label='Start Tree')
    
    # Plot goal tree
    if len(goal_nodes) > 0:
        ax.scatter(goal_nodes[:, 0], goal_nodes[:, 1], goal_nodes[:, 2], 
                  c='green', s=10, alpha=0.6, label='Goal Tree')
    
    # Plot path as a line
    if len(path) > 0:
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 
             c='red', linewidth=3, label=f'Path ({len(path)} nodes)')
        # Mark start and goal
        ax.scatter(path[0, 0], path[0, 1], path[0, 2], 
                  c='blue', s=200, marker='o', edgecolors='black', linewidth=2, label='Start')
        ax.scatter(path[-1, 0], path[-1, 1], path[-1, 2], 
                  c='green', s=200, marker='s', edgecolors='black', linewidth=2, label='Goal')
    
    # Labels and formatting
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D RRT Path Planning Visualization')
    ax.legend(loc='upper right')
    
    # Set equal aspect ratio for better visualization
    ax.set_box_aspect([1,1,1])
    
    plt.tight_layout()
    plt.savefig('rrt_visualization_3D.png', dpi=150, bbox_inches='tight')
    print("Saved visualization to rrt_visualization_3D.png")
    
    plt.show()

if __name__ == "__main__":
    visualize_3d_rrt()
