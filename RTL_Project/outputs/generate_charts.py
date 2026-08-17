from html import escape
from math import ceil, sqrt
from pathlib import Path


# Each row is one completed benchmark batch:
# (opt_runtime_s, con_runtime_s, opt_tree_kb, con_tree_kb,
#  opt_path_kb, con_path_kb)
DENSITIES = ["30%", "50%", "70%"]
PLATFORMS = ["Apple M4 Pro", "Intel Core Ultra 7"]
DATA = {
    "Apple M4 Pro": {
        "30%": [
            (0.001115, 0.001191, 28.367900, 7.786777, 0.191133, 1.395439),
            (0.001195, 0.001462, 29.199746, 8.407891, 0.194326, 1.425621),
            (0.001144, 0.001273, 28.351260, 7.827959, 0.190893, 1.405471),
        ],
        "50%": [
            (0.001823, 0.001904, 27.328604, 8.762637, 0.233156, 1.503691),
            (0.001707, 0.001808, 25.775146, 8.407715, 0.223002, 1.471734),
            (0.001738, 0.002011, 27.148848, 8.865127, 0.230414, 1.496145),
        ],
        "70%": [
            (0.002609, 0.003675, 19.521328, 9.389326, 0.241354, 1.577771),
            (0.002841, 0.003400, 20.927871, 8.967529, 0.250131, 1.563604),
            (0.002647, 0.003641, 20.069453, 9.081621, 0.246697, 1.572422),
        ],
    },
    "Intel Core Ultra 7": {
        "30%": [
            (0.002519, 0.001905, 28.790225, 8.037041, 0.192984, 1.414693),
            (0.002482, 0.001850, 29.210869, 8.028652, 0.197115, 1.411547),
            (0.002485, 0.001736, 28.381201, 7.752910, 0.191895, 1.404422),
            (0.002511, 0.001909, 28.998896, 8.122842, 0.191326, 1.403420),
            (0.002594, 0.001864, 28.773105, 8.012949, 0.193482, 1.407053),
        ],
        "50%": [
            (0.004085, 0.002493, 26.722656, 8.448428, 0.232119, 1.472479),
            (0.003934, 0.002747, 25.857529, 8.635596, 0.229430, 1.486441),
            (0.003919, 0.002945, 27.087109, 8.928887, 0.230232, 1.484291),
            (0.003852, 0.002577, 26.835342, 8.322725, 0.229125, 1.470838),
            (0.004016, 0.002642, 26.948281, 8.282734, 0.228264, 1.474371),
        ],
        "70%": [
            (0.005908, 0.004722, 18.242588, 7.795439, 0.237469, 1.506369),
            (0.006372, 0.005375, 19.614102, 9.295342, 0.244922, 1.557270),
            (0.006271, 0.005856, 19.626953, 9.953437, 0.247441, 1.587844),
            (0.005477, 0.004553, 18.266221, 8.297598, 0.236689, 1.532320),
            (0.006438, 0.004306, 19.567910, 8.437832, 0.243674, 1.523227),
        ],
    },
}

OPT_RUNTIME = 0
CON_RUNTIME = 1
OPT_TREE = 2
CON_TREE = 3
OPT_PATH = 4
CON_PATH = 5

OPT_TREE_COLOR = "#2563EB"
OPT_PATH_COLOR = "#93C5FD"
CON_TREE_COLOR = "#D1495B"
CON_PATH_COLOR = "#F4A6AE"
RTL_TREE_COLOR = "#0F8B6D"
RTL_PATH_COLOR = "#8BD3C7"
GRID_COLOR = "#D7DEE7"
AXIS_COLOR = "#52606D"
TEXT_COLOR = "#1F2933"
OUTPUT_DIR = Path(__file__).resolve().parent
RTL_RESULT_DIR = OUTPUT_DIR.parent / "01_RTL" / "result"
RTL_TREE_METADATA_KB = 4.0


def mean(values):
    return sum(values) / len(values)


def sample_sd(values):
    if len(values) < 2:
        return 0.0
    center = mean(values)
    return sqrt(sum((value - center) ** 2 for value in values) / (len(values) - 1))


def load_rtl_results():
    required = {
        "map", "seed", "success", "status", "cycles", "time_ms",
        "start_nodes", "goal_nodes", "path_nodes",
    }
    result = {}
    for density in DENSITIES:
        result_path = RTL_RESULT_DIR / "result_{0}.txt".format(density.rstrip("%"))
        if not result_path.exists():
            raise RuntimeError("missing RTL result file: {0}".format(result_path))

        header = None
        rows = []
        with result_path.open("r", encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line:
                    continue
                if line.startswith("#"):
                    candidate = line.lstrip("#").strip().split()
                    if required <= set(candidate):
                        header = candidate
                    continue
                if header is None:
                    raise RuntimeError("missing header in {0}".format(result_path))
                values = line.split()
                if len(values) != len(header):
                    raise RuntimeError("malformed row in {0}: {1}".format(result_path, line))
                row = dict(zip(header, values))
                rows.append({
                    "map": int(row["map"]),
                    "seed": int(row["seed"]),
                    "success": int(row["success"]),
                    "status": int(row["status"]),
                    "cycles": int(row["cycles"]),
                    "time_ms": float(row["time_ms"]),
                    "start_nodes": int(row["start_nodes"]),
                    "goal_nodes": int(row["goal_nodes"]),
                    "path_nodes": int(row["path_nodes"]),
                })
        if not rows:
            raise RuntimeError("RTL result file has no data: {0}".format(result_path))
        result[density] = rows
    return result


RTL_DATA = load_rtl_results()


def map_mean_sd(rows, value_function):
    values_by_map = {}
    for row in rows:
        values_by_map.setdefault(row["map"], []).append(value_function(row))
    map_means = [mean(values) for _, values in sorted(values_by_map.items())]
    return sample_sd(map_means)


def rtl_successful_rows(density):
    return [row for row in RTL_DATA[density] if row["success"] == 1]


def rtl_runtime_stats(density):
    rows = rtl_successful_rows(density)
    values = [row["time_ms"] for row in rows]
    return mean(values), map_mean_sd(rows, lambda row: row["time_ms"])


def rtl_memory_stats(density):
    rows = rtl_successful_rows(density)

    def tree_kb(row):
        return (row["start_nodes"] + row["goal_nodes"]) * 5.0 / 1024.0 + RTL_TREE_METADATA_KB

    def path_kb(row):
        return row["path_nodes"] * 3.0 / 1024.0

    tree_values = [tree_kb(row) for row in rows]
    path_values = [path_kb(row) for row in rows]
    total_values = [tree_kb(row) + path_kb(row) for row in rows]
    total_map_sd = map_mean_sd(rows, lambda row: tree_kb(row) + path_kb(row))
    return mean(tree_values), mean(path_values), mean(total_values), total_map_sd


def metric_values(platform, density, column):
    return [row[column] for row in DATA[platform][density]]


def runtime_stats(platform, density, column):
    values_ms = [value * 1000.0 for value in metric_values(platform, density, column)]
    return mean(values_ms), sample_sd(values_ms)


def memory_stats(platform, density, tree_column, path_column):
    rows = DATA[platform][density]
    tree_values = [row[tree_column] for row in rows]
    path_values = [row[path_column] for row in rows]
    total_values = [row[tree_column] + row[path_column] for row in rows]
    return mean(tree_values), mean(path_values), mean(total_values), sample_sd(total_values)


def svg_text(x, y, text, size=14, weight="400", anchor="middle", color=TEXT_COLOR,
             rotate=None):
    transform = ' transform="rotate({0} {1} {2})"'.format(rotate, x, y) if rotate is not None else ""
    return (
        '<text x="{0}" y="{1}" font-family="Arial, Helvetica, sans-serif" '
        'font-size="{2}" font-weight="{3}" text-anchor="{4}" fill="{5}"{6}>{7}</text>'
    ).format(x, y, size, weight, anchor, color, transform, escape(str(text)))


def svg_rect(x, y, width, height, color, stroke="none", stroke_width=0):
    return (
        '<rect x="{0:.2f}" y="{1:.2f}" width="{2:.2f}" height="{3:.2f}" '
        'fill="{4}" stroke="{5}" stroke-width="{6}"/>'
    ).format(x, y, width, max(0.0, height), color, stroke, stroke_width)


def svg_line(x1, y1, x2, y2, color, width=1):
    return (
        '<line x1="{0:.2f}" y1="{1:.2f}" x2="{2:.2f}" y2="{3:.2f}" '
        'stroke="{4}" stroke-width="{5}"/>'
    ).format(x1, y1, x2, y2, color, width)


def draw_error_bar(parts, x, center, spread, y_of, plot_max):
    lower = max(0.0, center - spread)
    upper = min(plot_max, center + spread)
    y_low = y_of(lower)
    y_high = y_of(upper)
    parts.append(svg_line(x, y_high, x, y_low, AXIS_COLOR, 1.5))
    parts.append(svg_line(x - 6, y_high, x + 6, y_high, AXIS_COLOR, 1.5))
    parts.append(svg_line(x - 6, y_low, x + 6, y_low, AXIS_COLOR, 1.5))
    return y_high


def chart_frame(width, height, title, subtitle, y_label):
    return [
        '<svg xmlns="http://www.w3.org/2000/svg" width="{0}" height="{1}" viewBox="0 0 {0} {1}">'.format(
            width, height
        ),
        '<rect width="100%" height="100%" fill="white"/>',
        svg_text(width / 2, 30, title, 26, "700"),
        svg_text(width / 2, 54, subtitle, 12, "400", color=AXIS_COLOR),
        svg_text(25, height / 2 + 10, y_label, 15, "600", rotate=-90),
    ]


def panel_geometry():
    width = 1240
    left = 82
    right = 38
    gap = 70
    panel_width = (width - left - right - gap) / 2.0
    return width, left, gap, panel_width


def add_panel_axes(parts, x0, panel_width, plot_top, plot_bottom, ticks, y_of,
                   show_tick_labels):
    for tick in ticks:
        y = y_of(tick)
        parts.append(svg_line(x0, y, x0 + panel_width, y, GRID_COLOR, 1))
        if show_tick_labels:
            parts.append(svg_text(x0 - 11, y + 5, "{0:g}".format(tick), 12,
                                  anchor="end", color=AXIS_COLOR))
    parts.append(svg_line(x0, plot_top, x0, plot_bottom, AXIS_COLOR, 1.5))
    parts.append(svg_line(x0, plot_bottom, x0 + panel_width, plot_bottom, AXIS_COLOR, 1.5))


def draw_speedup_chart(path):
    width, height = 1240, 520
    _, left, gap, panel_width = panel_geometry()
    plot_top, plot_bottom = 132, 428
    plot_height = plot_bottom - plot_top

    speedups = []
    for platform in PLATFORMS:
        for density in DENSITIES:
            rtl_time, _ = rtl_runtime_stats(density)
            opt_time, _ = runtime_stats(platform, density, OPT_RUNTIME)
            con_time, _ = runtime_stats(platform, density, CON_RUNTIME)
            speedups.extend([opt_time / rtl_time, con_time / rtl_time])
    y_max = max(2, int(ceil(max(speedups) * 1.12)))
    ticks = [index * 0.5 for index in range(0, y_max * 2 + 1)]

    def y_of(value):
        return plot_bottom - value / y_max * plot_height

    parts = chart_frame(
        width,
        height,
        "RTL Accelerator Speedup over CPU",
        "",
        "Speedup (x)",
    )
    legend_y = 84
    parts.extend([
        svg_rect(378, legend_y - 13, 18, 18, OPT_TREE_COLOR),
        svg_text(404, legend_y + 1, "vs Optimized CPU", 14, "600", anchor="start"),
        svg_rect(642, legend_y - 13, 18, 18, CON_TREE_COLOR),
        svg_text(668, legend_y + 1, "vs Conventional CPU", 14, "600", anchor="start"),
    ])

    bar_width = 46
    pair_offset = 27
    for platform_index, platform in enumerate(PLATFORMS):
        x0 = left + platform_index * (panel_width + gap)
        parts.append(svg_text(x0 + panel_width / 2, 116, platform, 18, "700"))
        add_panel_axes(parts, x0, panel_width, plot_top, plot_bottom, ticks, y_of,
                       platform_index == 0)
        baseline_y = y_of(1.0)
        parts.append(
            '<line x1="{0:.2f}" y1="{1:.2f}" x2="{2:.2f}" y2="{1:.2f}" '
            'stroke="{3}" stroke-width="1.5" stroke-dasharray="6 5"/>'.format(
                x0, baseline_y, x0 + panel_width, AXIS_COLOR
            )
        )
        cluster_width = panel_width / len(DENSITIES)

        for density_index, density in enumerate(DENSITIES):
            center_x = x0 + cluster_width * (density_index + 0.5)
            rtl_time, _ = rtl_runtime_stats(density)
            opt_time, _ = runtime_stats(platform, density, OPT_RUNTIME)
            con_time, _ = runtime_stats(platform, density, CON_RUNTIME)
            series = [
                (opt_time / rtl_time, center_x - pair_offset, OPT_TREE_COLOR),
                (con_time / rtl_time, center_x + pair_offset, CON_TREE_COLOR),
            ]
            for value, bar_center, color in series:
                bar_y = y_of(value)
                parts.append(svg_rect(bar_center - bar_width / 2, bar_y, bar_width,
                                      plot_bottom - bar_y, color))
                parts.append(svg_text(bar_center, bar_y - 9, "{0:.2f}x".format(value),
                                      11, "600"))

            parts.append(svg_text(center_x, plot_bottom + 27, density + " obstacles", 13, "600"))

    parts.append(svg_text(width / 2, height - 18, "Obstacle Density", 16, "600"))
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def draw_memory_chart(path):
    width, height = 1240, 520
    _, left, gap, panel_width = panel_geometry()
    plot_top, plot_bottom = 132, 404
    plot_height = plot_bottom - plot_top

    upper_values = []
    for platform in PLATFORMS:
        for density in DENSITIES:
            for tree_column, path_column in ((OPT_TREE, OPT_PATH), (CON_TREE, CON_PATH)):
                _, _, total, spread = memory_stats(platform, density, tree_column, path_column)
                upper_values.append(total + spread)
            _, _, rtl_total, rtl_spread = rtl_memory_stats(density)
            upper_values.append(rtl_total + rtl_spread)
    y_max = int(ceil(max(upper_values) * 1.08 / 5.0) * 5)
    ticks = list(range(0, y_max + 1, 5))

    def y_of(value):
        return plot_bottom - value / y_max * plot_height

    parts = chart_frame(
        width,
        height,
        "Logical Tree + Path Memory Usage",
        "",
        "Logical Memory (kB)",
    )
    legend_y = 84
    legend = [
        (300, OPT_TREE_COLOR, OPT_PATH_COLOR, "Optimized CPU"),
        (555, CON_TREE_COLOR, CON_PATH_COLOR, "Conventional CPU"),
        (830, RTL_TREE_COLOR, RTL_PATH_COLOR, "RTL accelerator"),
    ]
    for x, tree_color, path_color, label in legend:
        parts.append(svg_rect(x, legend_y - 8, 18, 13, tree_color, "white", 0.5))
        parts.append(svg_rect(x, legend_y - 13, 18, 5, path_color, "white", 0.5))
        parts.append(svg_text(x + 26, legend_y + 1, label, 14, "600", anchor="start"))

    bar_width = 34
    series_offset = 39
    for platform_index, platform in enumerate(PLATFORMS):
        x0 = left + platform_index * (panel_width + gap)
        parts.append(svg_text(x0 + panel_width / 2, 116, platform, 18, "700"))
        add_panel_axes(parts, x0, panel_width, plot_top, plot_bottom, ticks, y_of,
                       platform_index == 0)
        cluster_width = panel_width / len(DENSITIES)

        for density_index, density in enumerate(DENSITIES):
            center_x = x0 + cluster_width * (density_index + 0.5)
            opt = memory_stats(platform, density, OPT_TREE, OPT_PATH)
            con = memory_stats(platform, density, CON_TREE, CON_PATH)
            rtl = rtl_memory_stats(density)
            series = [
                (opt, center_x - series_offset, OPT_TREE_COLOR, OPT_PATH_COLOR),
                (con, center_x, CON_TREE_COLOR, CON_PATH_COLOR),
                (rtl, center_x + series_offset, RTL_TREE_COLOR, RTL_PATH_COLOR),
            ]
            for stats, bar_center, tree_color, path_color in series:
                tree_value, path_value, total_value, total_sd = stats
                tree_y = y_of(tree_value)
                total_y = y_of(total_value)
                parts.append(svg_rect(bar_center - bar_width / 2, tree_y, bar_width,
                                      plot_bottom - tree_y, tree_color))
                parts.append(svg_rect(bar_center - bar_width / 2, total_y, bar_width,
                                      tree_y - total_y, path_color, "white", 0.8))
                error_top = draw_error_bar(parts, bar_center, total_value, total_sd, y_of, y_max)
                parts.append(svg_text(bar_center, error_top - 8, "{0:.2f}".format(total_value),
                                      11, "600"))

            parts.append(svg_text(center_x, plot_bottom + 27, density + " obstacles", 13, "600"))
            parts.append(svg_text(center_x, plot_bottom + 51,
                                  "RTL/Opt = {0:.2f}x".format(rtl[2] / opt[2]),
                                  12, "700", color=AXIS_COLOR))

    parts.append(svg_text(width / 2, height - 18, "Obstacle Density", 16, "600"))
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def print_summary():
    print("RTL accelerator")
    for density in DENSITIES:
        rows = RTL_DATA[density]
        successful = rtl_successful_rows(density)
        rtl_time, _ = rtl_runtime_stats(density)
        rtl_memory = rtl_memory_stats(density)[2]
        print(
            "  {0}: success={1}/{2} latency={3:.6f} ms logical_mem={4:.6f} kB".format(
                density, len(successful), len(rows), rtl_time, rtl_memory
            )
        )

    for platform in PLATFORMS:
        print(platform)
        for density in DENSITIES:
            opt_runtime, _ = runtime_stats(platform, density, OPT_RUNTIME)
            con_runtime, _ = runtime_stats(platform, density, CON_RUNTIME)
            rtl_runtime, _ = rtl_runtime_stats(density)
            opt_memory = memory_stats(platform, density, OPT_TREE, OPT_PATH)[2]
            con_memory = memory_stats(platform, density, CON_TREE, CON_PATH)[2]
            print(
                "  {0}: RTL-vs-opt speedup={1:.6f}x RTL-vs-con speedup={2:.6f}x "
                "opt_mem={3:.6f} kB con_mem={4:.6f} kB".format(
                    density,
                    opt_runtime / rtl_runtime,
                    con_runtime / rtl_runtime,
                    opt_memory,
                    con_memory,
                )
            )


if __name__ == "__main__":
    draw_speedup_chart(OUTPUT_DIR / "run_time_comparison.svg")
    draw_memory_chart(OUTPUT_DIR / "tree_memory_comparison.svg")
    print_summary()
