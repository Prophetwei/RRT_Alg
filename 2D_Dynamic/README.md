# Dynamic-Map 2-D RRT Prototype

This directory preserves the dynamic-map prototype that explored path reuse and
tree invalidation as obstacles move between map frames. It predates the final
3-D accelerator and is included as design history rather than as a reported
benchmark target.

## Files

| File | Purpose |
| --- | --- |
| `rrt.cpp` | Dual-tree planner with map updates and node invalidation |
| `legacy_single_tree.cpp` | Earliest 100 x 100 single-tree dynamic prototype |
| `map_generator_dynamic.py` | Reproducible ten-frame moving-obstacle generator |
| `visualize_path.py` | Per-frame tree and path visualization |

Generated frame maps, tree snapshots, figures, and the `rrt` executable are
excluded from version control.

## Basic Use

```bash
python3 map_generator_dynamic.py
mkdir -p tree result
g++ -O3 -DNDEBUG -std=c++17 rrt.cpp -o rrt
./rrt
python3 visualize_path.py
```

Run the commands from this directory. The generator uses Python's standard
library; visualization requires NumPy and Matplotlib.
