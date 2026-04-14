# Úkol 2 - Vytvoření trasy (indoor_1)

Pro mapu `indoor_1` je trasa definována jako posloupnost waypointů v `public_vars.path` (vrací ji `plan_path.m`).

## Zadání splněno

- Trasa začíná v bodě `(2, 8.5)`.
- Trasa končí v cíli `(9, 9)`.
- Trasa neobsahuje jen přímky:
  - je použit Bézierův oblouk,
  - sinusový úsek,
  - kruhový oblouk.

## Kde je implementace

- `algorithms/path_planning/plan_path.m`
  - funkce `create_task3_task2_path()`.


