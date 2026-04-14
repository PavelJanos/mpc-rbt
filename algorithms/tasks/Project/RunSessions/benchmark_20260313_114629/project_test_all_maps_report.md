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
| indoor_2 | 1.200 | 0.800 | -2.576 (-0.820*pi) | 9.200 | 9.200 | 1.0 |
## Souhrn

| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_2 | 0.10 | 1 | 1 | 0 | 8 | 1700.0 | 5.010 | 352.572 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_2 | 3 | 1.200 | 0.800 | -2.576 (-0.820*pi) | 9.200 | 9.200 | goal | 1 | 1700 | 0.495 | 31.020 | 228.914 |
| indoor_2 | 1 | 8.800 | 6.200 | -2.486 (-0.791*pi) | 0.800 | 0.800 | timeout | 0 | 2050 | 8.202 | 32.587 | 311.655 |
| indoor_2 | 4 | 9.200 | 3.800 | 1.671 (0.532*pi) | 0.800 | 9.200 | timeout | 0 | 2050 | 5.624 | 26.462 | 309.833 |
| indoor_2 | 2 | 9.200 | 8.600 | 1.006 (pi/3) | 0.800 | 3.800 | timeout | 0 | 2050 | 4.228 | 23.776 | 532.200 |
| indoor_2 | 5 | 0.800 | 3.800 | 2.155 (2*pi/3) | 9.200 | 3.400 | timeout | 0 | 2050 | 5.530 | 30.608 | 350.601 |
| indoor_2 | 7 | 1.200 | 6.400 | -2.732 (-0.870*pi) | 7.200 | 5.000 | timeout | 0 | 2050 | 4.506 | 25.890 | 357.598 |
| indoor_2 | 6 | 0.800 | 9.200 | 2.405 (3*pi/4) | 9.200 | 6.400 | timeout | 0 | 2050 | 7.638 | 30.564 | 421.445 |
| indoor_2 | 8 | 7.000 | 2.800 | -2.274 (-3*pi/4) | 2.800 | 5.800 | timeout | 0 | 2050 | 1.648 | 21.930 | 299.443 |
| indoor_2 | 9 | 3.200 | 8.200 | 1.642 (pi/2) | 7.000 | 2.600 | timeout | 0 | 2050 | 4.277 | 20.223 | 233.447 |
| indoor_2 | 10 | 5.000 | 6.200 | -0.508 (-0.162*pi) | 9.200 | 0.800 | wall | 0 | 1674 | 7.953 | 22.536 | 480.587 |
