# Task6/Task2 - Odstup od prekazek

- Pozadovany minimalni odstup: `0.20 m`
- Metoda: inflace prekazek v occupancy mape o 0.20 m v planneru.

## Vysledky

| Planner | Trasa nalezena | Odstup splnen | Min odstup [m] | Delka [m] | Waypointy |
|---|---:|---:|---:|---:|---:|
| astar | 1 | 1 | 0.300 | 20.968 | 93 |
| dijkstra | 1 | 1 | 0.354 | 20.968 | 93 |
| greedy | 1 | 1 | 0.300 | 24.988 | 116 |

## Vystupy
- `task2_clearance_validation.png`
- `task2_clearance_table.csv`
- `task2_clearance_report.md`
