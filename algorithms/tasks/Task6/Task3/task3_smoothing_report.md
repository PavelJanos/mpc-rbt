# Task6/Task3 - Porovnani vyhlazovacich metod

- Testovana mapa: `indoor_1`, start `[2.00, 1.00]`, cil `[9.00, 9.00]`
- Pozadovany odstup od prekazek: `>= 0.20 m`

## Tabulka vysledku

| Metoda | Trasa nalezena | Odstup splnen | Waypointy | Delka [m] | Min odstup [m] | Sum zmen smeru [rad] | Cas [s] | Score |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| iterative | 1 | 1 | 93 | 20.968 | 0.354 | 10.210 | 0.0047 | 0.7750 |
| shortcut | 1 | 1 | 6 | 19.488 | 0.200 | 4.613 | 0.0499 | 0.6839 |
| spline | 1 | 1 | 279 | 21.038 | 0.360 | 15.288 | 0.0070 | 0.9140 |
| chaikin | 1 | 1 | 372 | 20.823 | 0.354 | 10.210 | 0.0062 | 0.7745 |

## Doporuceni
- Nejvhodnejsi metoda podle score: `shortcut`
- Vliv parametru:
  - Heading variation [rad] = soucet absolutnich zmen smeroveho uhlu podél trasy (mensi hodnota znamena plynulejsi trasu).
  - Iterative: vyssi `beta` vice vyhlazuje, vyssi `alpha` vice drzi puvodni trasu.
  - Shortcut: agresivne zkracuje trasu, ale muze ponechat ostrejsi zmeny smeru.
  - Spline: nejhladsi geometrie, ale muze se blizit prekazkam bez bezpecnostni kontroly.
  - Chaikin: plynule zaobluje rohy, pocet bodu roste s poctem iteraci.

## Vystupy
- `task3_smoothing_comparison.png`
- `task3_smoothing_comparison.csv`
- `task3_smoothing_report.md`
