import statistics
import sys
from pathlib import Path


METADATA_COLUMNS = {
    "map",
    "seed",
    "opt_first",
    "repeats",
    "opt_success",
    "con_success",
    "opt_deterministic",
    "con_deterministic",
}


def success_mode(header):
    columns = set(header)
    if {"opt_success", "con_success", "repeats"} <= columns:
        return "explicit_repeat_counts"
    if {"opt_success", "con_success"} <= columns:
        return "explicit_flags"
    if {"opt_path_mem", "con_path_mem"} <= columns:
        return "legacy_path_memory"
    raise ValueError(
        "result file has no usable success fields; expected "
        "opt_success/con_success or opt_path_mem/con_path_mem"
    )


def row_succeeded(values, mode):
    if mode == "explicit_repeat_counts":
        repeats = int(values["repeats"])
        return (
            repeats > 0
            and int(values["opt_success"]) == repeats
            and int(values["con_success"]) == repeats
        )
    if mode == "explicit_flags":
        return int(values["opt_success"]) > 0 and int(values["con_success"]) > 0

    # Legacy files did not record success explicitly.  A non-empty path is the
    # only reliable success marker available for both implementations.
    return float(values["opt_path_mem"]) > 0.0 and float(values["con_path_mem"]) > 0.0


def read_metrics(file_path):
    with file_path.open("r", encoding="utf-8") as handle:
        lines = [line.strip() for line in handle if line.strip()]

    if not lines:
        raise ValueError("result file is empty")

    header = lines[0].lstrip("#").split()
    mode = success_mode(header)
    metric_names = [name for name in header if name not in METADATA_COLUMNS]
    metrics = {name: [] for name in metric_names}

    total_rows = len(lines) - 1
    successful_rows = 0
    malformed_rows = 0

    for line in lines[1:]:
        row = line.split()
        if len(row) != len(header):
            malformed_rows += 1
            continue

        values = dict(zip(header, row))
        try:
            succeeded = row_succeeded(values, mode)
        except (KeyError, ValueError):
            malformed_rows += 1
            continue

        if not succeeded:
            continue

        try:
            numeric_values = {
                name: float(values[name]) for name in metric_names
            }
        except (KeyError, ValueError):
            malformed_rows += 1
            continue

        for name, value in numeric_values.items():
            metrics[name].append(value)
        successful_rows += 1

    return metrics, total_rows, successful_rows, malformed_rows, mode


def summarize_metrics(metrics):
    for name, values in metrics.items():
        print(f"{name}:")
        print(f"  average: {statistics.mean(values):.6f}")
        print(f"  median:  {statistics.median(values):.6f}")
        print(f"  best:    {min(values):.6f}")
        print(f"  worst:   {max(values):.6f}")
        print()


def main():
    file_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("results_seed42.txt")

    if not file_path.exists():
        print(f"File not found: {file_path}")
        return 1

    try:
        metrics, total, successful, malformed, mode = read_metrics(file_path)
    except ValueError as error:
        print(f"Cannot analyze {file_path}: {error}")
        return 2

    print(f"success filter : {mode}")
    print(f"rows accepted  : {successful}/{total}")
    print(f"rows excluded  : {total - successful - malformed}")
    print(f"rows malformed : {malformed}\n")

    if successful == 0:
        print("No paired successful rows to summarize.")
        return 3

    summarize_metrics(metrics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
