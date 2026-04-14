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
- mode cases: `reuse`
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
| indoor_2 | 0.60 | 6 | 1 | 0 | 3 | 1191.0 | 2.828 | 230.723 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_2 | 3 | 1.200 | 0.800 | -2.576 (-0.820*pi) | 9.200 | 9.200 | goal | 1 | 1621 | 0.491 | 30.923 | 224.188 |
| indoor_2 | 4 | 9.200 | 3.800 | 1.671 (0.532*pi) | 0.800 | 9.200 | timeout | 0 | 2050 | 4.904 | 25.312 | 253.737 |
| indoor_2 | 1 | 8.800 | 6.200 | -2.486 (-0.791*pi) | 0.800 | 0.800 | wall | 0 | 2035 | 7.448 | 31.962 | 276.904 |
| indoor_2 | 2 | 9.200 | 8.600 | 1.006 (pi/3) | 0.800 | 3.800 | goal | 1 | 1498 | 0.489 | 26.785 | 373.495 |
| indoor_2 | 7 | 1.200 | 6.400 | -2.732 (-0.870*pi) | 7.200 | 5.000 | goal | 1 | 1392 | 0.496 | 22.795 | 176.038 |
| indoor_2 | 8 | 7.000 | 2.800 | -2.274 (-3*pi/4) | 2.800 | 5.800 | goal | 1 | 711 | 0.491 | 14.934 | 87.522 |
| indoor_2 | 10 | 5.000 | 6.200 | -0.508 (-0.162*pi) | 9.200 | 0.800 | goal | 1 | 789 | 0.499 | 16.337 | 55.857 |
| indoor_2 | 9 | 3.200 | 8.200 | 1.642 (pi/2) | 7.000 | 2.600 | goal | 1 | 1135 | 0.498 | 18.471 | 83.015 |
| indoor_2 | 5 | 0.800 | 3.800 | 2.155 (2*pi/3) | 9.200 | 3.400 | timeout | 0 | 2050 | 5.326 | 29.237 | 362.511 |
| indoor_2 | 6 | 0.800 | 9.200 | 2.405 (3*pi/4) | 9.200 | 6.400 | timeout | 0 | 2050 | 7.638 | 30.564 | 413.961 |
