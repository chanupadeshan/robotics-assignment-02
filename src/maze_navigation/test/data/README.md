`winding_costmap.pgm` is a raw uint8 Nav2 global-costmap snapshot (not an occupancy map),
128 x 128, 0.025 m/cell, origin (-1.6, -1.6), after correcting the robot localization.
It reproduces a reachable ~6.8 m maze route that exhausts stock Jazzy NavFn's
512 half-cell path-extraction steps. The regression preserves the exact costs,
checks that extended extraction succeeds, and keeps disconnected goals rejected.
Source evidence: workspace reports/top-route-diagnosis-after.json and top-route-after.npz.
