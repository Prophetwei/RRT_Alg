import argparse
import secrets
from pathlib import Path

import numpy as np


DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "../PE_test"
DEFAULT_OUTPUT = "map.txt"


class CompatibleRNG:
    def __init__(self, seed):
        if hasattr(np.random, "default_rng"):
            self._rng = np.random.default_rng(seed)
            self.backend = "Generator"
        else:
            # RandomState only accepts a 32-bit seed on older NumPy releases.
            self._rng = np.random.RandomState(int(seed) & 0xFFFFFFFF)
            self.backend = "RandomState"

    def integers(self, low, high=None, size=None):
        if hasattr(self._rng, "integers"):
            return self._rng.integers(low, high=high, size=size)
        return self._rng.randint(low, high=high, size=size)

    def choice(self, *args, **kwargs):
        return self._rng.choice(*args, **kwargs)

    def shuffle(self, values):
        self._rng.shuffle(values)


def make_rng(seed):
    return CompatibleRNG(seed)


def quantile(values, probability):
    if hasattr(np, "quantile"):
        return np.quantile(values, probability)
    return np.percentile(values, probability * 100.0)


def normalize_density(value):
    text = str(value).strip()
    explicit_percent = text.endswith("%")
    if explicit_percent:
        text = text[:-1].strip()

    try:
        density = float(text)
    except ValueError as error:
        raise ValueError(f"invalid density: {value!r}") from error

    if not np.isfinite(density):
        raise ValueError("density must be a finite number")
    if explicit_percent or density >= 1.0:
        density /= 100.0
    if not 0.0 < density < 1.0:
        raise ValueError(
            "density must be 0-1 as a fraction or 0-100 as a percentage"
        )
    return density


def parse_density(value):
    try:
        return normalize_density(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def clear_safe_zones(grid, safe_size):
    if safe_size == 0:
        return
    grid[:safe_size, :safe_size, :safe_size] = False
    grid[-safe_size:, -safe_size:, -safe_size:] = False


def make_valid_mask(size, safe_size):
    mask = np.ones((size, size, size), dtype=bool)
    clear_safe_zones(mask, safe_size)
    return mask


def make_random_monotonic_path(size, rng):
    steps = np.repeat(np.arange(3, dtype=np.int8), size - 1)
    rng.shuffle(steps)
    path = np.empty((steps.size + 1, 3), dtype=np.int16)
    position = np.zeros(3, dtype=np.int16)
    path[0] = position

    for index, axis in enumerate(steps, start=1):
        position[int(axis)] += 1
        path[index] = position

    return path


def make_corridor_mask(size, path, radius):
    corridor = np.zeros((size, size, size), dtype=bool)
    for x, y, z in path:
        x0, x1 = max(int(x) - radius, 0), min(int(x) + radius + 1, size)
        y0, y1 = max(int(y) - radius, 0), min(int(y) + radius + 1, size)
        z0, z1 = max(int(z) - radius, 0), min(int(z) + radius + 1, size)
        corridor[x0:x1, y0:y1, z0:z1] = True
    return corridor


def validate_guaranteed_path(grid, path):
    if path is None:
        return
    size = grid.shape[0]
    if not np.array_equal(path[0], (0, 0, 0)):
        raise RuntimeError("guaranteed path does not start at (0,0,0)")
    if not np.array_equal(path[-1], (size - 1, size - 1, size - 1)):
        raise RuntimeError("guaranteed path does not end at the goal")
    if not np.all(np.abs(np.diff(path.astype(np.int32), axis=0)).sum(axis=1) == 1):
        raise RuntimeError("guaranteed path is not 6-connected")
    if np.any(grid[path[:, 0], path[:, 1], path[:, 2]]):
        raise RuntimeError("guaranteed path intersects an obstacle")


def propose_random_block(size, region_bins, target_region, rng, min_block, max_block):
    dimensions = rng.integers(min_block, max_block + 1, size=3)
    region_width = size // region_bins
    region_lower = np.asarray(target_region) * region_width
    region_upper = region_lower + region_width
    anchor = np.asarray(
        [rng.integers(region_lower[i], region_upper[i]) for i in range(3)],
        dtype=int,
    )

    # The anchor is random inside an under-filled region.  Its random position
    # within the block lets obstacles cross region boundaries naturally.
    starts = np.array(
        [
            rng.integers(int(anchor[axis] - d + 1), int(anchor[axis] + 1))
            for axis, d in enumerate(dimensions)
        ],
        dtype=int,
    )
    lower = np.maximum(starts, 0)
    upper = np.minimum(starts + dimensions, size)
    return lower, upper


def measure_block_gains(grid, valid_mask, lower, upper, region_bins):
    size = grid.shape[0]
    region_width = size // region_bins
    first_region = lower // region_width
    last_region = (upper - 1) // region_width
    gains = np.zeros((region_bins, region_bins, region_bins), dtype=np.int64)

    for rx in range(int(first_region[0]), int(last_region[0]) + 1):
        x_slice = slice(
            max(int(lower[0]), rx * region_width),
            min(int(upper[0]), (rx + 1) * region_width),
        )
        for ry in range(int(first_region[1]), int(last_region[1]) + 1):
            y_slice = slice(
                max(int(lower[1]), ry * region_width),
                min(int(upper[1]), (ry + 1) * region_width),
            )
            for rz in range(int(first_region[2]), int(last_region[2]) + 1):
                z_slice = slice(
                    max(int(lower[2]), rz * region_width),
                    min(int(upper[2]), (rz + 1) * region_width),
                )
                region = (x_slice, y_slice, z_slice)
                gains[rx, ry, rz] = np.count_nonzero(
                    valid_mask[region] & ~grid[region]
                )

    return gains


def choose_underfilled_region(region_counts, target_region_counts, rng):
    deficits = np.maximum(target_region_counts - region_counts, 0.0)
    weights = deficits.ravel()
    if weights.sum() == 0.0:
        flat_index = int(rng.integers(weights.size))
    else:
        flat_index = int(rng.choice(weights.size, p=weights / weights.sum()))
    return np.unravel_index(flat_index, region_counts.shape)


def apply_block(grid, valid_mask, lower, upper):
    block = tuple(slice(int(lower[i]), int(upper[i])) for i in range(3))
    np.logical_or(grid[block], valid_mask[block], out=grid[block])


def generate_candidate(
    size,
    target_density,
    rng,
    min_block,
    max_block,
    valid_mask,
    region_bins,
    ensure_path,
    corridor_radius,
    placement_trials,
    max_attempts,
):
    grid = np.zeros((size, size, size), dtype=bool)
    if ensure_path:
        guaranteed_path = make_random_monotonic_path(size, rng)
        corridor_mask = make_corridor_mask(size, guaranteed_path, corridor_radius)
    else:
        guaranteed_path = None
        corridor_mask = np.zeros_like(grid)

    placement_mask = valid_mask & ~corridor_mask
    target_count = int(round((size ** 3) * target_density))
    available_regions = region_sums(valid_mask, region_bins)
    placement_regions = region_sums(placement_mask, region_bins)
    effective_density = target_count / int(np.count_nonzero(valid_mask))
    target_region_counts = available_regions * effective_density
    if target_count > int(np.count_nonzero(placement_mask)):
        raise ValueError("target density leaves no room for the protected corridor")
    if np.any(target_region_counts > placement_regions):
        raise ValueError(
            "corridor occupies too much of a density region; reduce density, "
            "corridor radius, or increase region size"
        )
    region_counts = np.zeros_like(available_regions, dtype=np.int64)
    obstacle_count = 0

    for attempts in range(1, max_attempts + 1):
        best_proposal = None
        best_gains = None
        best_error = None

        for _ in range(placement_trials):
            target_region = choose_underfilled_region(
                region_counts, target_region_counts, rng
            )
            lower, upper = propose_random_block(
                size,
                region_bins,
                target_region,
                rng,
                min_block,
                max_block,
            )
            gains = measure_block_gains(
                grid, placement_mask, lower, upper, region_bins
            )
            if gains.sum() == 0:
                continue

            projected_counts = region_counts + gains
            projected_error = float(
                np.sum(
                    ((projected_counts - target_region_counts) ** 2)
                    / available_regions
                )
            )
            if best_error is None or projected_error < best_error:
                best_proposal = (lower, upper)
                best_gains = gains
                best_error = projected_error

        if best_proposal is None:
            continue

        apply_block(grid, placement_mask, *best_proposal)
        region_counts += best_gains
        obstacle_count += int(best_gains.sum())
        if obstacle_count >= target_count:
            validate_guaranteed_path(grid, guaranteed_path)
            return (
                grid,
                attempts,
                guaranteed_path,
                int(np.count_nonzero(corridor_mask)),
            )

    raise RuntimeError(
        f"Could not reach density {target_density:.3f} after {max_attempts} blocks"
    )


def masked_density(grid, valid_mask, region):
    available = int(np.count_nonzero(valid_mask[region]))
    if available == 0:
        return 0.0
    return float(np.count_nonzero(grid[region])) / available


def region_sums(array, bins):
    size = array.shape[0]
    if size % bins != 0:
        raise ValueError("size must be divisible by region_bins")
    width = size // bins
    reshaped = array.reshape(bins, width, bins, width, bins, width)
    return reshaped.sum(axis=(1, 3, 5))


def analyze_distribution(grid, valid_mask, region_bins):
    obstacle_regions = region_sums(grid, region_bins)
    available_regions = region_sums(valid_mask, region_bins)
    regional_density = obstacle_regions / available_regions

    size = grid.shape[0]
    half = size // 2
    root_width = size // region_bins

    axis_deltas = []
    for axis in range(3):
        low = [slice(None)] * 3
        high = [slice(None)] * 3
        low[axis] = slice(0, half)
        high[axis] = slice(half, size)
        axis_deltas.append(
            abs(
                masked_density(grid, valid_mask, tuple(low))
                - masked_density(grid, valid_mask, tuple(high))
            )
        )

    start_octant = (slice(0, half), slice(0, half), slice(0, half))
    goal_octant = (slice(half, size), slice(half, size), slice(half, size))
    start_root = (
        slice(0, root_width),
        slice(0, root_width),
        slice(0, root_width),
    )
    goal_root = (
        slice(size - root_width, size),
        slice(size - root_width, size),
        slice(size - root_width, size),
    )

    opposite_delta = np.abs(
        regional_density - regional_density[::-1, ::-1, ::-1]
    )
    global_density = float(np.count_nonzero(grid)) / grid.size
    regional_std = float(np.std(regional_density))
    axis_delta = float(max(axis_deltas))
    octant_delta = abs(
        masked_density(grid, valid_mask, start_octant)
        - masked_density(grid, valid_mask, goal_octant)
    )
    start_root_density = masked_density(grid, valid_mask, start_root)
    goal_root_density = masked_density(grid, valid_mask, goal_root)
    root_delta = abs(start_root_density - goal_root_density)
    opposite_mean_delta = float(np.mean(opposite_delta))
    mirror_mismatch = float(np.mean(grid != grid[::-1, ::-1, ::-1]))

    # All terms are absolute density differences.  Selecting the lowest score
    # improves statistical uniformity without copying or mirroring obstacles.
    score = (
        regional_std
        + axis_delta
        + octant_delta
        + root_delta
        + opposite_mean_delta
    )

    return {
        "global_density": global_density,
        "regional_mean": float(np.mean(regional_density)),
        "regional_std": regional_std,
        "regional_min": float(np.min(regional_density)),
        "regional_max": float(np.max(regional_density)),
        "axis_delta": axis_delta,
        "octant_delta": float(octant_delta),
        "start_root_density": float(start_root_density),
        "goal_root_density": float(goal_root_density),
        "root_delta": float(root_delta),
        "opposite_mean_delta": opposite_mean_delta,
        "mirror_mismatch": mirror_mismatch,
        "score": float(score),
    }


def print_candidate_metrics(index, attempts, metrics):
    print(
        f"Candidate {index}: blocks={attempts}, density={metrics['global_density']:.4f}, "
        f"regions={metrics['regional_mean']:.4f}+/-{metrics['regional_std']:.4f}, "
        f"range=[{metrics['regional_min']:.4f}, {metrics['regional_max']:.4f}]"
    )
    print(
        f"  axis_delta={metrics['axis_delta']:.4f}, "
        f"octant_delta={metrics['octant_delta']:.4f}, "
        f"root_density={metrics['start_root_density']:.4f}/{metrics['goal_root_density']:.4f}, "
        f"root_delta={metrics['root_delta']:.4f}"
    )
    print(
        f"  opposite_region_delta={metrics['opposite_mean_delta']:.4f}, "
        f"mirror_mismatch={metrics['mirror_mismatch']:.4f}, "
        f"score={metrics['score']:.4f}"
    )
    if metrics["path_guaranteed"]:
        print(
            f"  guaranteed_path_steps={metrics['path_steps']}, "
            f"protected_corridor_voxels={metrics['corridor_voxels']}"
        )


def audit_random_subcubes(
    grid,
    valid_mask,
    rng,
    sample_size,
    rounds,
    samples_per_round,
):
    if sample_size > grid.shape[0]:
        raise ValueError("sample_size cannot exceed map size")

    all_densities = []
    round_means = []
    size = grid.shape[0]
    if size % sample_size != 0:
        raise ValueError("map size must be divisible by sample_size")
    tiles_per_axis = size // sample_size
    tile_count = tiles_per_axis ** 3

    for _ in range(rounds):
        densities = []
        tile_indices = rng.choice(
            tile_count,
            size=samples_per_round,
            replace=samples_per_round > tile_count,
        )
        for tile_index in tile_indices:
            tile = np.asarray(
                np.unravel_index(int(tile_index), (tiles_per_axis,) * 3)
            )
            origin = tile * sample_size
            region = tuple(
                slice(int(origin[i]), int(origin[i] + sample_size))
                for i in range(3)
            )
            densities.append(masked_density(grid, valid_mask, region))
        all_densities.extend(densities)
        round_means.append(float(np.mean(densities)))

    samples = np.asarray(all_densities)
    means = np.asarray(round_means)
    print(
        f"Stratified sub-cube audit: {rounds} rounds x {samples_per_round} samples, "
        f"cube={sample_size}^3"
    )
    print(
        f"  sample mean={samples.mean():.4f}, std={samples.std():.4f}, "
        f"p05={quantile(samples, 0.05):.4f}, "
        f"p95={quantile(samples, 0.95):.4f}"
    )
    print(
        f"  round mean={means.mean():.4f}, round-to-round std={means.std():.4f}"
    )


def write_map(grid, output_path):
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_name(output_path.name + ".tmp")

    with temporary_path.open("w", encoding="ascii") as output_file:
        for x in range(grid.shape[0]):
            yz = np.argwhere(grid[x])
            if yz.size == 0:
                continue
            xyz = np.column_stack((np.full(yz.shape[0], x, dtype=int), yz))
            np.savetxt(output_file, xyz, fmt="%d")

    temporary_path.replace(output_path)


def write_guaranteed_path(path, output_path):
    if path is None:
        return None
    output_path = Path(output_path)
    path_output = output_path.with_name(
        f"{output_path.stem}_guaranteed_path.txt"
    )
    temporary_path = path_output.with_name(path_output.name + ".tmp")
    np.savetxt(str(temporary_path), path, fmt="%d")
    temporary_path.replace(path_output)
    return path_output


def generate_dense_3d_map(
    filename=DEFAULT_OUTPUT,
    size=256,
    target_percentage=0.30,
    seed=None,
    candidates=5,
    region_bins=8,
    min_block=8,
    max_block=20,
    safe_size=8,
    ensure_path=True,
    corridor_radius=3,
    write_path=True,
    max_attempts=10000,
    placement_trials=32,
    sample_size=32,
    sample_rounds=10,
    samples_per_round=64,
    audit_only=False,
):
    target_percentage = normalize_density(target_percentage)
    if not 0 <= safe_size < size // 2:
        raise ValueError("safe_size must be between 0 and size/2")
    if not 1 <= min_block <= max_block <= size:
        raise ValueError("block sizes must satisfy 1 <= min <= max <= size")
    if not 0 <= corridor_radius < size:
        raise ValueError("corridor_radius must be between 0 and size-1")
    if candidates < 1:
        raise ValueError("candidates must be at least 1")
    if placement_trials < 1:
        raise ValueError("placement_trials must be at least 1")
    if region_bins < 1 or size % region_bins != 0:
        raise ValueError("size must be divisible by region_bins")
    if sample_size < 1 or size % sample_size != 0:
        raise ValueError("size must be divisible by sample_size")
    if sample_rounds < 1 or samples_per_round < 1:
        raise ValueError("sample counts must be at least 1")

    valid_mask = make_valid_mask(size, safe_size)
    target_count = int(round((size ** 3) * target_percentage))
    if target_count > int(np.count_nonzero(valid_mask)):
        raise ValueError("target density leaves no room for the requested safe zones")

    if seed is None:
        seed = secrets.randbits(63)
    rng = make_rng(seed)
    print(f"Map seed: {seed}")
    print(f"NumPy RNG backend: {rng.backend}")
    print(
        f"Generating {candidates} random candidates at {size}^3, "
        f"target density={target_percentage:.2%}"
    )

    best_grid = None
    best_metrics = None
    best_index = 0
    best_path = None

    for index in range(1, candidates + 1):
        grid, attempts, guaranteed_path, corridor_voxels = generate_candidate(
            size,
            target_percentage,
            rng,
            min_block,
            max_block,
            valid_mask,
            region_bins,
            ensure_path,
            corridor_radius,
            placement_trials,
            max_attempts,
        )
        metrics = analyze_distribution(grid, valid_mask, region_bins)
        metrics["path_guaranteed"] = guaranteed_path is not None
        metrics["path_steps"] = 0 if guaranteed_path is None else len(guaranteed_path) - 1
        metrics["corridor_voxels"] = corridor_voxels
        print_candidate_metrics(index, attempts, metrics)

        if best_metrics is None or metrics["score"] < best_metrics["score"]:
            best_grid = grid
            best_metrics = metrics
            best_index = index
            best_path = guaranteed_path

    print(f"Selected candidate {best_index} with score={best_metrics['score']:.4f}")
    audit_rng = make_rng(seed ^ 0x5A17D3C9)
    audit_random_subcubes(
        best_grid,
        valid_mask,
        audit_rng,
        sample_size,
        sample_rounds,
        samples_per_round,
    )

    if audit_only:
        print("Audit-only mode: map file was not changed")
    else:
        write_map(best_grid, filename)
        print(
            f"Wrote {np.count_nonzero(best_grid)} obstacles to "
            f"'{Path(filename).resolve()}'"
        )
        if write_path and best_path is not None:
            path_output = write_guaranteed_path(best_path, filename)
            print(f"Wrote guaranteed path certificate to '{path_output.resolve()}'")

    return best_grid, best_metrics


def generate_dense_3d_maps(
    output_dir=DEFAULT_OUTPUT_DIR,
    count=1,
    **kwargs,
):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    base_seed = kwargs.get("seed")
    for index in range(count):
        if base_seed is None:
            map_seed = None
        else:
            map_seed = base_seed + index
        generate_dense_3d_map(
            filename=output_dir / "map.txt",
            seed=map_seed,
            **{key: value for key, value in kwargs.items() if key != "seed"},
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate statistically uniform random 3D block obstacles"
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="folder for batch output maps",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="number of maps to generate in batch mode",
    )
    parser.add_argument("--size", type=int, default=256)
    parser.add_argument(
        "--density",
        type=parse_density,
        default=0.30,
        metavar="DENSITY",
        help="obstacle density, e.g. 0.30, 30, or 30%% (default: 30%%)",
    )
    parser.add_argument("--seed", type=int)
    parser.add_argument("--candidates", type=int, default=5)
    parser.add_argument(
        "--region-bins",
        type=int,
        default=8,
        help="number of density-balancing regions on each axis",
    )
    parser.add_argument("--min-block", type=int, default=8)
    parser.add_argument("--max-block", type=int, default=20)
    parser.add_argument("--safe-size", type=int, default=8)
    parser.add_argument(
        "--corridor-radius",
        type=int,
        default=3,
        help="protected free-space radius around the guaranteed random path",
    )
    parser.add_argument(
        "--no-ensure-path",
        action="store_false",
        dest="ensure_path",
        help="disable the guaranteed random start-to-goal corridor",
    )
    parser.add_argument(
        "--no-write-path",
        action="store_false",
        dest="write_path",
        help="do not write the guaranteed path certificate",
    )
    parser.set_defaults(ensure_path=True, write_path=False)
    parser.add_argument("--max-attempts", type=int, default=10000)
    parser.add_argument(
        "--placement-trials",
        type=int,
        default=32,
        help="random block proposals considered at each placement",
    )
    parser.add_argument("--sample-size", type=int, default=32)
    parser.add_argument("--sample-rounds", type=int, default=10)
    parser.add_argument("--samples-per-round", type=int, default=64)
    parser.add_argument(
        "--audit-only",
        action="store_true",
        help="run generation and statistics without replacing the output map",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.count <= 1:
        generate_dense_3d_map(
            filename=args.output,
            size=args.size,
            target_percentage=args.density,
            seed=args.seed,
            candidates=args.candidates,
            region_bins=args.region_bins,
            min_block=args.min_block,
            max_block=args.max_block,
            safe_size=args.safe_size,
            ensure_path=args.ensure_path,
            corridor_radius=args.corridor_radius,
            write_path=args.write_path,
            max_attempts=args.max_attempts,
            placement_trials=args.placement_trials,
            sample_size=args.sample_size,
            sample_rounds=args.sample_rounds,
            samples_per_round=args.samples_per_round,
            audit_only=args.audit_only,
        )
    else:
        generate_dense_3d_maps(
            output_dir=args.output_dir,
            count=args.count,
            size=args.size,
            target_percentage=args.density,
            seed=args.seed,
            candidates=args.candidates,
            region_bins=args.region_bins,
            min_block=args.min_block,
            max_block=args.max_block,
            safe_size=args.safe_size,
            ensure_path=args.ensure_path,
            corridor_radius=args.corridor_radius,
            write_path=args.write_path,
            max_attempts=args.max_attempts,
            placement_trials=args.placement_trials,
            sample_size=args.sample_size,
            sample_rounds=args.sample_rounds,
            samples_per_round=args.samples_per_round,
            audit_only=args.audit_only,
        )
