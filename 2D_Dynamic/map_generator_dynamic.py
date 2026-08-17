import math
import os
import random


def generate_moving_maps(
    num_maps=10,
    size=1024,
    num_obstacles=180,
    obstacle_size_range=(60, 220),
    movement_direction=(1, 0),
    movement_speed=4,
    safety_margin=24,
    seed=42,
    coverage_ratio=0.82,
    min_obstacle_gap=4,
):
    """
    Generate 10 complex dynamic maps with mixed obstacle types.

    The generator creates rectangles, circles, rotated rectangles, and corridor-like
    barriers that move across frames, while preserving a guaranteed open route from
    start to goal.
    """
    random.seed(seed)
    os.makedirs("map", exist_ok=True)

    start = (0, 0)
    goal = (size - 1, size - 1)
    min_size, max_size = obstacle_size_range
    max_displacement = movement_speed * max(0, num_maps - 1)
    dense_cell = max(24, min_size + min_obstacle_gap)

    def clamp(value, low, high):
        return max(low, min(high, value))

    def in_start_goal_zone(x1, y1, x2, y2):
        zones = [start, goal]
        for zx, zy in zones:
            if not (x2 + safety_margin < zx - safety_margin or x1 - safety_margin > zx + safety_margin or
                    y2 + safety_margin < zy - safety_margin or y1 - safety_margin > zy + safety_margin):
                return True
        return False

    def overlap(a1, b1, a2, b2, obstacles, gap=10):
        for obstacle in obstacles:
            if obstacle["type"] == "rect":
                ox1, oy1, ox2, oy2 = obstacle["x1"], obstacle["y1"], obstacle["x2"], obstacle["y2"]
            elif obstacle["type"] == "circle":
                ox1 = obstacle["cx"] - obstacle["radius"]
                oy1 = obstacle["cy"] - obstacle["radius"]
                ox2 = obstacle["cx"] + obstacle["radius"]
                oy2 = obstacle["cy"] + obstacle["radius"]
            elif obstacle["type"] == "rot_rect":
                margin = max(obstacle["w"], obstacle["h"]) // 2 + 2
                ox1 = obstacle["cx"] - margin
                oy1 = obstacle["cy"] - margin
                ox2 = obstacle["cx"] + margin
                oy2 = obstacle["cy"] + margin
            else:
                if obstacle["horizontal"]:
                    ox1 = obstacle["cx"] - obstacle["length"] // 2
                    oy1 = obstacle["cy"] - obstacle["thickness"] // 2
                    ox2 = obstacle["cx"] + obstacle["length"] // 2
                    oy2 = obstacle["cy"] + obstacle["thickness"] // 2
                else:
                    ox1 = obstacle["cx"] - obstacle["thickness"] // 2
                    oy1 = obstacle["cy"] - obstacle["length"] // 2
                    ox2 = obstacle["cx"] + obstacle["thickness"] // 2
                    oy2 = obstacle["cy"] + obstacle["length"] // 2

            if not (a2 + gap < ox1 or a1 - gap > ox2 or b2 + gap < oy1 or b1 - gap > oy2):
                return True
        return False

    def respect_min_gap(x1, y1, x2, y2, placed_boxes, gap=min_obstacle_gap):
        for px1, py1, px2, py2 in placed_boxes:
            if not (x2 + gap < px1 or x1 - gap > px2 or y2 + gap < py1 or y1 - gap > py2):
                return False
        return True

    def add_rect(map_matrix, x1, y1, x2, y2, value=1):
        for yy in range(max(0, y1), min(size, y2 + 1)):
            for xx in range(max(0, x1), min(size, x2 + 1)):
                map_matrix[yy][xx] = value

    def add_circle(map_matrix, cx, cy, radius, value=1):
        r2 = radius * radius
        for yy in range(max(0, cy - radius), min(size, cy + radius + 1)):
            for xx in range(max(0, cx - radius), min(size, cx + radius + 1)):
                if (xx - cx) ** 2 + (yy - cy) ** 2 <= r2:
                    map_matrix[yy][xx] = value

    def add_rotated_rect(map_matrix, cx, cy, w, h, angle, value=1):
        hw = w / 2.0
        hh = h / 2.0
        ca = math.cos(-angle)
        sa = math.sin(-angle)
        bb_w = int(abs(w * math.cos(angle)) + abs(h * math.sin(angle))) + 2
        bb_h = int(abs(w * math.sin(angle)) + abs(h * math.cos(angle))) + 2
        x1 = cx - bb_w // 2
        x2 = cx + bb_w // 2
        y1 = cy - bb_h // 2
        y2 = cy + bb_h // 2
        for yy in range(max(0, y1), min(size, y2 + 1)):
            for xx in range(max(0, x1), min(size, x2 + 1)):
                rx = xx - cx
                ry = yy - cy
                ux = rx * ca - ry * sa
                uy = rx * sa + ry * ca
                if abs(ux) <= hw and abs(uy) <= hh:
                    map_matrix[yy][xx] = value

    def fill_dense_background(map_matrix, frame, placed_boxes):
        # Build a dense but irregular obstacle field with random placement.
        target_count = int((size * size / (dense_cell * dense_cell)) * coverage_ratio)
        attempts = target_count * 30
        placed_count = 0

        for _ in range(attempts):
            if placed_count >= target_count:
                break

            shape_type = random.choice(["rect", "circle", "rot_rect"])
            x1 = random.randint(0, size - 1)
            y1 = random.randint(0, size - 1)

            if shape_type == "rect":
                w = random.randint(max(12, min_size // 2), max(24, max_size // 2))
                h = random.randint(max(12, min_size // 2), max(24, max_size // 2))
                x2 = clamp(x1 + w, 0, size - 1)
                y2 = clamp(y1 + h, 0, size - 1)
                if in_start_goal_zone(x1, y1, x2, y2):
                    continue
                if not respect_min_gap(x1, y1, x2, y2, placed_boxes):
                    continue
                add_rect(map_matrix, x1, y1, x2, y2)
                placed_boxes.append((x1, y1, x2, y2))
                placed_count += 1

            elif shape_type == "circle":
                radius = random.randint(max(8, min_size // 4), max(18, max_size // 4))
                cx = clamp(x1, radius, size - radius - 1)
                cy = clamp(y1, radius, size - radius - 1)
                x2 = cx + radius
                y2 = cy + radius
                x0 = cx - radius
                y0 = cy - radius
                if in_start_goal_zone(x0, y0, x2, y2):
                    continue
                if not respect_min_gap(x0, y0, x2, y2, placed_boxes):
                    continue
                add_circle(map_matrix, cx, cy, radius)
                placed_boxes.append((x0, y0, x2, y2))
                placed_count += 1

            else:
                w = random.randint(max(16, min_size // 2), max(32, max_size // 2))
                h = random.randint(max(16, min_size // 2), max(32, max_size // 2))
                angle = random.uniform(0.0, math.pi)
                cx = clamp(x1, w // 2 + 1, size - w // 2 - 2)
                cy = clamp(y1, h // 2 + 1, size - h // 2 - 2)
                bb_w = int(abs(w * math.cos(angle)) + abs(h * math.sin(angle))) + 4
                bb_h = int(abs(w * math.sin(angle)) + abs(h * math.cos(angle))) + 4
                x0 = cx - bb_w // 2
                y0 = cy - bb_h // 2
                x2 = cx + bb_w // 2
                y2 = cy + bb_h // 2
                if in_start_goal_zone(x0, y0, x2, y2):
                    continue
                if not respect_min_gap(x0, y0, x2, y2, placed_boxes):
                    continue
                add_rotated_rect(map_matrix, cx, cy, w, h, angle)
                placed_boxes.append((x0, y0, x2, y2))
                placed_count += 1

    base_obstacles = []
    for _ in range(num_obstacles):
        for _attempt in range(60):
            shape_type = random.choice(["rect", "circle", "rot_rect", "barrier"])
            if shape_type == "circle":
                radius = random.randint(10, max_size // 2)
                cx = random.randint(60 + radius, size - 60 - radius - 1)
                cy = random.randint(60 + radius, size - 60 - radius - 1)
                x1, y1, x2, y2 = cx - radius, cy - radius, cx + radius, cy + radius
                if in_start_goal_zone(x1, y1, x2, y2) or overlap(x1, y1, x2, y2, base_obstacles):
                    continue
                base_obstacles.append({"type": "circle", "cx": cx, "cy": cy, "radius": radius})
                break

            if shape_type == "rot_rect":
                w = random.randint(min_size, max_size)
                h = random.randint(min_size, max_size)
                angle = random.uniform(0.0, math.pi)
                margin = max(w, h) // 2 + 60
                cx = random.randint(margin, size - margin - 1)
                cy = random.randint(margin, size - margin - 1)
                x1, y1, x2, y2 = cx - w // 2, cy - h // 2, cx + w // 2, cy + h // 2
                if in_start_goal_zone(x1, y1, x2, y2) or overlap(x1, y1, x2, y2, base_obstacles):
                    continue
                base_obstacles.append({"type": "rot_rect", "cx": cx, "cy": cy, "w": w, "h": h, "angle": angle})
                break

            if shape_type == "barrier":
                horizontal = random.choice([True, False])
                thickness = random.randint(12, 30)
                length = random.randint(size // 6, size // 2)
                if horizontal:
                    cx = random.randint(80, size - 80)
                    cy = random.randint(120, size - 120)
                    x1, y1 = cx - length // 2, cy - thickness // 2
                    x2, y2 = cx + length // 2, cy + thickness // 2
                else:
                    cx = random.randint(120, size - 120)
                    cy = random.randint(80, size - 80)
                    x1, y1 = cx - thickness // 2, cy - length // 2
                    x2, y2 = cx + thickness // 2, cy + length // 2
                if in_start_goal_zone(x1, y1, x2, y2) or overlap(x1, y1, x2, y2, base_obstacles):
                    continue
                base_obstacles.append({"type": "barrier", "cx": cx, "cy": cy, "horizontal": horizontal, "thickness": thickness, "length": length})
                break

            width = random.randint(min_size, max_size)
            height = random.randint(min_size, max_size)
            x1 = random.randint(60, size - width - 60)
            y1 = random.randint(60, size - height - 60)
            x2 = x1 + width
            y2 = y1 + height
            if in_start_goal_zone(x1, y1, x2, y2) or overlap(x1, y1, x2, y2, base_obstacles):
                continue
            base_obstacles.append({"type": "rect", "x1": x1, "y1": y1, "x2": x2, "y2": y2})
            break

    dx_per_frame = movement_direction[0] * movement_speed
    dy_per_frame = movement_direction[1] * movement_speed
    orthogonal = (-movement_direction[1], movement_direction[0])

    for frame in range(num_maps):
        map_matrix = [[0 for _ in range(size)] for _ in range(size)]
        placed_boxes = []
        fill_dense_background(map_matrix, frame, placed_boxes)
        offset_x = int(dx_per_frame * frame)
        offset_y = int(dy_per_frame * frame)
        wobble = int(round(math.sin(frame * 0.8) * 6))
        wobble_x = orthogonal[0] * wobble
        wobble_y = orthogonal[1] * wobble

        for obstacle in base_obstacles:
            if obstacle["type"] == "rect":
                new_x1 = obstacle["x1"] + offset_x + wobble_x
                new_y1 = obstacle["y1"] + offset_y + wobble_y
                new_x2 = obstacle["x2"] + offset_x + wobble_x
                new_y2 = obstacle["y2"] + offset_y + wobble_y
                if not respect_min_gap(new_x1, new_y1, new_x2, new_y2, placed_boxes):
                    continue
                add_rect(map_matrix, new_x1, new_y1, new_x2, new_y2)
                placed_boxes.append((new_x1, new_y1, new_x2, new_y2))

            elif obstacle["type"] == "circle":
                cx = obstacle["cx"] + offset_x + wobble_x
                cy = obstacle["cy"] + offset_y + wobble_y
                radius = obstacle["radius"]
                if not respect_min_gap(cx - radius, cy - radius, cx + radius, cy + radius, placed_boxes):
                    continue
                add_circle(map_matrix, cx, cy, radius)
                placed_boxes.append((cx - radius, cy - radius, cx + radius, cy + radius))

            elif obstacle["type"] == "rot_rect":
                cx = obstacle["cx"] + offset_x + wobble_x
                cy = obstacle["cy"] + offset_y + wobble_y
                bb_w = int(abs(obstacle["w"] * math.cos(obstacle["angle"])) + abs(obstacle["h"] * math.sin(obstacle["angle"]))) + 4
                bb_h = int(abs(obstacle["w"] * math.sin(obstacle["angle"])) + abs(obstacle["h"] * math.cos(obstacle["angle"]))) + 4
                if not respect_min_gap(cx - bb_w // 2, cy - bb_h // 2, cx + bb_w // 2, cy + bb_h // 2, placed_boxes):
                    continue
                add_rotated_rect(map_matrix, cx, cy, obstacle["w"], obstacle["h"], obstacle["angle"])
                placed_boxes.append((cx - bb_w // 2, cy - bb_h // 2, cx + bb_w // 2, cy + bb_h // 2))

            elif obstacle["type"] == "barrier":
                cx = obstacle["cx"] + offset_x + wobble_x
                cy = obstacle["cy"] + offset_y + wobble_y
                length = obstacle["length"]
                thickness = obstacle["thickness"]
                if obstacle["horizontal"]:
                    x1, y1 = cx - length // 2, cy - thickness // 2
                    x2, y2 = cx + length // 2, cy + thickness // 2
                else:
                    x1, y1 = cx - thickness // 2, cy - length // 2
                    x2, y2 = cx + thickness // 2, cy + length // 2
                if not respect_min_gap(x1, y1, x2, y2, placed_boxes):
                    continue
                add_rect(map_matrix, x1, y1, x2, y2)
                placed_boxes.append((x1, y1, x2, y2))

        for y in range(0, min(size, safety_margin + 1)):
            for x in range(0, min(size, safety_margin + 1)):
                map_matrix[y][x] = 0
                map_matrix[size - 1 - y][size - 1 - x] = 0

        filename = f"map/map{frame}.txt"
        with open(filename, "w") as f:
            for row in map_matrix:
                f.write(" ".join(map(str, row)) + "\n")


if __name__ == "__main__":
    generate_moving_maps(
        num_maps=10,
        size=1024,
        num_obstacles=180,
        obstacle_size_range=(60, 220),
        movement_direction=(1, 0),
        movement_speed=4,
        safety_margin=50,
        seed=42,
        coverage_ratio=0.82,
        min_obstacle_gap=4,
    )