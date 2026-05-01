import random
import math

def generate_map_with_gaps(filename="map/map.txt", size=1024, num_blocks=60, min_gap=10,
                           num_circles=40, num_rot_rects=30, corridor_count=40, border_margin=8,
                           border_obstacle_width=0, border_obstacle_gap_prob=0.12,
                           edge_obstacle_count=20, edge_max_size=180):
    """
    Generate a map with mixed obstacle types: axis-aligned blocks, circles,
    rotated rectangles and corridor lines. A border margin is reserved so that
    obstacles do not touch the map edge.
    """
    # initialize empty map (rows x cols)
    map_matrix = [[0 for _ in range(size)] for _ in range(size)]
    placed_blocks = []  # store bounding boxes as (x1,y1,x2,y2)

    def bbox_conflicts(x1, y1, x2, y2):
        # check with min_gap padding
        for (bx1, by1, bx2, by2) in placed_blocks:
            if not (x2 + min_gap < bx1 or x1 - min_gap > bx2 or y2 + min_gap < by1 or y1 - min_gap > by2):
                return True
        return False

    def clamp_range(v, low, high):
        return max(low, min(high, v))

    # --- Perimeter obstacles: fill a band along the border, leave random gaps ---
    if border_obstacle_width > 0:
        for yy in range(size):
            for xx in range(size):
                if xx < border_obstacle_width or xx >= size - border_obstacle_width or yy < border_obstacle_width or yy >= size - border_obstacle_width:
                    if random.random() > border_obstacle_gap_prob:
                        map_matrix[yy][xx] = 1
        # record perimeter bbox for conflict checks
        placed_blocks.append((0, 0, size - 1, size - 1))

    # --- Edge-attached obstacles: place some obstacles that touch a map edge ---
    for _ in range(edge_obstacle_count):
        # choose which edge: 0=left,1=right,2=top,3=bottom
        edge = random.randint(0, 3)
        bw = random.randint(6, min(edge_max_size, size // 2))
        bh = random.randint(6, min(edge_max_size, size // 2))
        for attempt in range(40):
            if edge == 0:  # left
                x1 = 0
                y1 = random.randint(border_margin, size - bh - border_margin - 1)
                x2 = x1 + bw
                y2 = y1 + bh
            elif edge == 1:  # right
                x2 = size - 1
                x1 = x2 - bw
                y1 = random.randint(border_margin, size - bh - border_margin - 1)
                y2 = y1 + bh
            elif edge == 2:  # top
                y2 = size - 1
                y1 = y2 - bh
                x1 = random.randint(border_margin, size - bw - border_margin - 1)
                x2 = x1 + bw
            else:  # bottom
                y1 = 0
                x1 = random.randint(border_margin, size - bw - border_margin - 1)
                x2 = x1 + bw
                y2 = y1 + bh

            if x1 < 0: x1 = 0
            if y1 < 0: y1 = 0
            if x2 >= size: x2 = size - 1
            if y2 >= size: y2 = size - 1

            if bbox_conflicts(x1 - min_gap, y1 - min_gap, x2 + min_gap, y2 + min_gap):
                continue
            for yy in range(y1, y2 + 1):
                for xx in range(x1, x2 + 1):
                    map_matrix[yy][xx] = 1
            placed_blocks.append((x1, y1, x2, y2))
            break


    # --- Axis-aligned rectangular blocks (existing behaviour, but inside margin) ---
    for _ in range(num_blocks):
        block_w = random.randint(20, 120)
        block_h = random.randint(20, 120)
        for attempt in range(80):
            x1 = random.randint(border_margin, size - block_w - border_margin - 1)
            y1 = random.randint(border_margin, size - block_h - border_margin - 1)
            x2, y2 = x1 + block_w, y1 + block_h
            if not bbox_conflicts(x1, y1, x2, y2):
                for yy in range(y1, y2):
                    for xx in range(x1, x2):
                        map_matrix[yy][xx] = 1
                placed_blocks.append((x1, y1, x2, y2))
                break

    # --- Circles ---
    for _ in range(num_circles):
        r = random.randint(8, 60)
        for attempt in range(60):
            cx = random.randint(border_margin + r, size - border_margin - r - 1)
            cy = random.randint(border_margin + r, size - border_margin - r - 1)
            x1, y1 = cx - r, cy - r
            x2, y2 = cx + r, cy + r
            if bbox_conflicts(x1, y1, x2, y2):
                continue
            rsq = r * r
            for yy in range(y1, y2 + 1):
                if yy < 0 or yy >= size: continue
                for xx in range(x1, x2 + 1):
                    if xx < 0 or xx >= size: continue
                    dx = xx - cx
                    dy = yy - cy
                    if dx * dx + dy * dy <= rsq:
                        map_matrix[yy][xx] = 1
            placed_blocks.append((x1, y1, x2, y2))
            break

    # --- Rotated rectangles ---
    for _ in range(num_rot_rects):
        w = random.randint(20, 140)
        h = random.randint(10, 100)
        ang = random.uniform(0, math.pi)
        for attempt in range(60):
            cx = random.randint(border_margin + max(w, h), size - border_margin - max(w, h) - 1)
            cy = random.randint(border_margin + max(w, h), size - border_margin - max(w, h) - 1)
            # bounding box of rotated rectangle
            cos_a = abs(math.cos(ang))
            sin_a = abs(math.sin(ang))
            bb_w = int(w * cos_a + h * sin_a)
            bb_h = int(w * sin_a + h * cos_a)
            x1, y1 = cx - bb_w // 2, cy - bb_h // 2
            x2, y2 = x1 + bb_w, y1 + bb_h
            if bbox_conflicts(x1, y1, x2, y2):
                continue
            # fill rotated rect: rotate each candidate point back and test
            hw = w / 2.0
            hh = h / 2.0
            ca = math.cos(-ang)
            sa = math.sin(-ang)
            for yy in range(y1, y2 + 1):
                if yy < 0 or yy >= size: continue
                for xx in range(x1, x2 + 1):
                    if xx < 0 or xx >= size: continue
                    rx = xx - cx
                    ry = yy - cy
                    ux = rx * ca - ry * sa
                    uy = rx * sa + ry * ca
                    if abs(ux) <= hw and abs(uy) <= hh:
                        map_matrix[yy][xx] = 1
            placed_blocks.append((x1, y1, x2, y2))
            break

    # --- Corridors (thick line segments connecting two random points) ---
    for _ in range(corridor_count):
        thickness = random.randint(2, 8)
        for attempt in range(40):
            x1 = random.randint(border_margin, size - border_margin - 1)
            y1 = random.randint(border_margin, size - border_margin - 1)
            x2 = random.randint(border_margin, size - border_margin - 1)
            y2 = random.randint(border_margin, size - border_margin - 1)
            bb_x1 = min(x1, x2) - thickness - min_gap
            bb_y1 = min(y1, y2) - thickness - min_gap
            bb_x2 = max(x1, x2) + thickness + min_gap
            bb_y2 = max(y1, y2) + thickness + min_gap
            if bbox_conflicts(bb_x1, bb_y1, bb_x2, bb_y2):
                continue
            # Bresenham-like thick line: step along the line and fill circle of thickness
            dx = x2 - x1
            dy = y2 - y1
            dist = math.hypot(dx, dy)
            if dist == 0:
                continue
            steps = int(dist) + 1
            for i in range(steps + 1):
                t = i / max(1, steps)
                cx = int(x1 + dx * t)
                cy = int(y1 + dy * t)
                rr = thickness
                for yy in range(cy - rr, cy + rr + 1):
                    if yy < 0 or yy >= size: continue
                    for xx in range(cx - rr, cx + rr + 1):
                        if xx < 0 or xx >= size: continue
                        if (xx - cx) ** 2 + (yy - cy) ** 2 <= rr * rr:
                            map_matrix[yy][xx] = 1
            placed_blocks.append((bb_x1, bb_y1, bb_x2, bb_y2))
            break

    # --- Save to file ---
    with open(filename, "w") as f:
        for row in map_matrix:
            f.write(" ".join(map(str, row)) + "\n")

    print(f"Map saved to {filename} with {len(placed_blocks)} placed obstacle bboxes.")


if __name__ == "__main__":
    # Example generation: larger variety
    generate_map_with_gaps(size=1024, num_blocks=80, min_gap=6, num_circles=60, num_rot_rects=40, corridor_count=60, border_margin=12)