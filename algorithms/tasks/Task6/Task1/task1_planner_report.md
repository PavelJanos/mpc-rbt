# Task6/Task1 - Porovnani planovacich algoritmu

Pouzite algoritmy: `A*`, `Dijkstra`, `Greedy Best-First`.

Mapa: `indoor_1`, start `[2.00, 1.00]`, cil `[9.00, 9.00]`.

## Vysledky

| Algoritmus | Uspech | Pocet waypointu | Delka trasy [m] | Chyba k cili [m] | Cas [s] | Score |
|---|---:|---:|---:|---:|---:|---:|
| astar | 1 | 89 | 20.168 | 0.000 | 0.0080 | 0.7604 |
| dijkstra | 1 | 89 | 20.168 | 0.000 | 0.0129 | 0.8742 |
| greedy | 1 | 114 | 24.588 | 0.000 | 0.0037 | 0.7847 |

## Doporuceny default
- Doporuceny planner: `astar`
- Duvod: nejlepsi kompromis delky trasy a vypocetniho casu pri uspesnem nalezeni cesty.

## Jak prepinat planner
V `public_vars` nastavte:

```matlab
public_vars.force_grid_planner = true;
public_vars.path_planner_mode = 'astar';    % nebo 'dijkstra' / 'greedy'
```
