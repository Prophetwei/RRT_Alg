import random
import numpy as np

def generate_map_3d(filename="map/map_3D.txt", size=1000, num_blocks=400, min_gap=5):
    # Initialize 3D matrix (Voxel Grid) with 0
    # Using numpy for easier handling of large 3D arrays
    map_matrix = np.zeros((size, size, size), dtype=int)
    placed_blocks = []

    print(f"Generating 3D map: {size}x{size}x{size}...")

    for _ in range(num_blocks):
        # 1. Randomize dimensions for the 3D box
        block_w = random.randint(10, 25)
        block_h = random.randint(10, 25)
        block_d = random.randint(10, 25)
        
        # 2. Attempt to place the block without overlap
        for attempt in range(100):
            x1 = random.randint(0, size - block_w - 1)
            y1 = random.randint(0, size - block_h - 1)
            z1 = random.randint(0, size - block_d - 1)
            x2, y2, z2 = x1 + block_w, y1 + block_h, z1 + block_d
            
            # Check overlap with existing blocks plus a safety gap
            is_overlapping = False
            for (bx1, by1, bz1, bx2, by2, bz2) in placed_blocks:
                if not (x2 + min_gap < bx1 or x1 - min_gap > bx2 or 
                        y2 + min_gap < by1 or y1 - min_gap > by2 or
                        z2 + min_gap < bz1 or z1 - min_gap > bz2):
                    is_overlapping = True
                    break
            
            if not is_overlapping:
                # 3. Fill the 3D block in the matrix
                map_matrix[x1:x2, y1:y2, z1:z2] = 1
                placed_blocks.append((x1, y1, z1, x2, y2, z2))
                break

    # 4. Save to file
    # Note: Flattening a 3D array for a text file can make it very large.
    # We save it as X Y Z coordinates of obstacles to keep the file size manageable.
    with open(filename, "w") as f:
        # Write only the coordinates of occupied voxels (1s)
        for z in range(size):
            for y in range(size):
                for x in range(size):
                    if map_matrix[x, y, z] == 1:
                        f.write(f"{x} {y} {z}\n")
            
    print(f"3D Map saved to {filename}. Total obstacle voxels: {np.sum(map_matrix)}")

if __name__ == "__main__":
    generate_map_3d(size=500, num_blocks=1000, min_gap=8)