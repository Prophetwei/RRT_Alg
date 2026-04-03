import os

def generate_moving_maps(num_maps=10, size=100, step_size=2):
    # Ensure the directory is clean
    for i in range(num_maps):
        map_matrix = [[0 for _ in range(size)] for _ in range(size)]
        
        # Calculate the shifting position of the obstacle
        # The wall starts at x=40 and moves 2 pixels right per file
        offset = i * step_size
        wall_x_start = 40 + offset
        wall_x_end = 45 + offset
        
        # Create a vertical wall with a small gap
        gap_y_start = 45
        gap_y_end = 55
        
        for y in range(10, 90):
            if not (gap_y_start < y < gap_y_end):
                for x in range(wall_x_start, wall_x_end):
                    if x < size: # Boundary safety
                        map_matrix[y][x] = 1
        
        # Save each map to a unique filename
        filename = f"map{i}.txt"
        with open(filename, "w") as f:
            for row in map_matrix:
                f.write(" ".join(map(str, row)) + "\n")
        
        print(f"Generated {filename}: Obstacle at x={wall_x_start}")

if __name__ == "__main__":
    generate_moving_maps()