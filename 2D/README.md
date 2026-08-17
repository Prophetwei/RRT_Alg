# Static 2-D RRT Prototype

This directory preserves the static-map 2-D prototype developed before the
final 3-D hardware study. It demonstrates dual-tree growth, branch extension,
path extraction, and tree visualization on a 1024 x 1024 occupancy map.

This prototype is retained for project-history traceability. Its measurements
are not used in the final 3-D accelerator results.

## Files

| File | Purpose |
| --- | --- |
| `rrt.cpp` | Dual-tree 2-D planner |
| `map_generator.py` | Static obstacle-map generator |
| `verify.py` | Early path-checking utility |
| `visualize_path.py` | Tree and path visualization |

Generated maps, tree dumps, figures, and the `rrt` executable are intentionally
excluded from version control.

## Basic Use

```bash
python3 map_generator.py
mkdir -p tree
g++ -O3 -DNDEBUG -std=c++17 rrt.cpp -o rrt
./rrt
python3 visualize_path.py
```

Run these commands from this directory because the prototype uses relative
`map/` and `tree/` paths. Python visualization requires NumPy and Matplotlib.
