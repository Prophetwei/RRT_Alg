import random

def generate_map_with_gaps(filename="map/map.txt", size=1000, num_blocks=60, min_gap=10):
    map_matrix = [[0 for _ in range(size)] for _ in range(size)]
    placed_blocks = []

    for _ in range(num_blocks):
        # 1. Define randomized size for the block
        block_w = random.randint(30, 80)
        block_h = random.randint(30, 80)
        
        # 2. Try multiple times to find a spot that doesn't overlap
        for attempt in range(50): 
            x1 = random.randint(0, size - block_w - 1)
            y1 = random.randint(0, size - block_h - 1)
            x2, y2 = x1 + block_w, y1 + block_h
            
            # Check against previously placed blocks for the 'min_gap'
            is_overlapping = False
            for (bx1, by1, bx2, by2) in placed_blocks:
                # Add min_gap to the collision box check
                if not (x2 + min_gap < bx1 or x1 - min_gap > bx2 or 
                        y2 + min_gap < by1 or y1 - min_gap > by2):
                    is_overlapping = True
                    break
            
            if not is_overlapping:
                # 3. Fill the block
                for y in range(y1, y2):
                    for x in range(x1, x2):
                        map_matrix[y][x] = 1
                placed_blocks.append((x1, y1, x2, y2))
                break

    with open(filename, "w") as f:
        for row in map_matrix:
            f.write(" ".join(map(str, row)) + "\n")
            
    print(f"Map saved to {filename} with {len(placed_blocks)} non-overlapping blocks.")

if __name__ == "__main__":
    generate_map_with_gaps(size=1024, num_blocks=100, min_gap=10)