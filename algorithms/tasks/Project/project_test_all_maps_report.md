# Project - Benchmark jedne mapy

Benchmark pouziva stejnou hlavni smycku jako `main`, ale bez GUI a bez `waitforbuttonpress`.

Pouzite nastaveni:
- planner: `A*`
- smoothing: `chaikin`
- controller: `pure pursuit`
- max velocity: `1.0 m/s`
- benchmark mode: `test`
- view mode: `true`
- stop mode: `all`
- selected test map: `indoor_2`
- repeats na mapu: `10`
- timeout: `2050` kroku (`205.0 s` pri `Ts = 0.1 s`)
- mode cases: `regenerate`
- start/cil/theta: nahodne generovane pro kazdy beh nebo znovu pouzite z CSV podle mode
- validace pripadu: clearance + existence cesty pres planner
- cases file: `project_test_all_maps_cases_test_indoor_2.csv`

## Konfigurace behu

| Mapa | Priklad start x [m] | Priklad start y [m] | Priklad start theta [rad] | Priklad cil x [m] | Priklad cil y [m] | Max vel [m/s] |
|---|---:|---:|---:|---:|---:|---:|
| indoor_2 | 8.800 | 6.200 | -2.486 (-0.791*pi) | 0.800 | 0.800 | 1.0 |
## Souhrn

| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_2 | 0.60 | 6 | 1 | 0 | 3 | 1194.5 | 2.287 | 77.542 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_2 | 1 | 8.800 | 6.200 | -2.486 (-0.791*pi) | 0.800 | 0.800 | goal | 1 | 1650 | 0.497 | 27.118 | 81.308 |
| indoor_2 | 2 | 9.200 | 8.600 | 1.006 (pi/3) | 0.800 | 3.800 | goal | 1 | 1319 | 0.486 | 26.353 | 61.663 |
| indoor_2 | 3 | 1.200 | 0.800 | -2.576 (-0.820*pi) | 9.200 | 9.200 | timeout | 0 | 2050 | 0.732 | 32.152 | 94.353 |
| indoor_2 | 4 | 9.200 | 3.800 | 1.671 (0.532*pi) | 0.800 | 9.200 | timeout | 0 | 2050 | 6.589 | 22.369 | 107.332 |
| indoor_2 | 5 | 0.800 | 3.800 | 2.155 (2*pi/3) | 9.200 | 3.400 | wall | 0 | 1142 | 8.128 | 14.543 | 63.918 |
| indoor_2 | 6 | 0.800 | 9.200 | 2.405 (3*pi/4) | 9.200 | 6.400 | goal | 1 | 1133 | 0.485 | 20.084 | 65.278 |
| indoor_2 | 7 | 1.200 | 6.400 | -2.732 (-0.870*pi) | 7.200 | 5.000 | goal | 1 | 914 | 0.496 | 14.571 | 51.762 |
| indoor_2 | 8 | 7.000 | 2.800 | -2.274 (-3*pi/4) | 2.800 | 5.800 | timeout | 0 | 2050 | 4.472 | 24.815 | 120.569 |
| indoor_2 | 9 | 3.200 | 8.200 | 1.642 (pi/2) | 7.000 | 2.600 | goal | 1 | 823 | 0.489 | 16.927 | 46.426 |
| indoor_2 | 10 | 5.000 | 6.200 | -0.508 (-0.162*pi) | 9.200 | 0.800 | goal | 1 | 1328 | 0.494 | 23.873 | 82.806 |
