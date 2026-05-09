import numpy as np

def verify_path(map_file="map/map.txt", path_file="tree/path"):
    # 1. Load the Map into a set for O(1) lookup
    obstacles = set()
    with open(map_file, "r") as f:
        next(f) # Skip header
        for line in f:
            obstacles.add(tuple(map(int, line.split())))

    # 2. Load the Path
    path = np.loadtxt(path_file)
    
    # 3. Check for collisions at every node
    for i, point in enumerate(path):
        voxel = tuple(point.astype(int))
        if voxel in obstacles:
            print(f"❌ COLLISION detected at node {i}: {voxel}")
            return False

    # 4. Check for continuity (Step size check)
    for i in range(len(path) - 1):
        dist = np.linalg.norm(path[i+1] - path[i])
        if dist > 6.0: # Should be close to your SEARCH_RANGE
            print(f"❌ DISCONTINUITY: Gap of {dist:.2f} between node {i} and {i+1}")
            return False

    print("✅ Path Verified: Continuous and Collision-Free!")
    return True

verify_path()