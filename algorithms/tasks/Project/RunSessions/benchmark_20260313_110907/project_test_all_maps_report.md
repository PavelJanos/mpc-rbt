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
| indoor_2 | 0.00 | 0 | 2 | 0 | 8 | NaN | 6.955 | 267.457 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_2 | 1 | 8.800 | 6.200 | -2.486 (-0.791*pi) | 0.800 | 0.800 | timeout | 0 | 2050 | 4.783 | 33.074 | 285.413 |
| indoor_2 | 2 | 9.200 | 8.600 | 1.006 (pi/3) | 0.800 | 3.800 | timeout | 0 | 2050 | 6.216 | 29.144 | 309.697 |
| indoor_2 | 3 | 1.200 | 0.800 | -2.576 (-0.820*pi) | 9.200 | 9.200 | timeout | 0 | 2050 | 11.225 | 23.275 | 409.339 |
| indoor_2 | 4 | 9.200 | 3.800 | 1.671 (0.532*pi) | 0.800 | 9.200 | timeout | 0 | 2050 | 10.185 | 20.733 | 273.420 |
| indoor_2 | 5 | 0.800 | 3.800 | 2.155 (2*pi/3) | 9.200 | 3.400 | timeout | 0 | 2050 | 5.911 | 10.627 | 163.368 |
| indoor_2 | 6 | 0.800 | 9.200 | 2.405 (3*pi/4) | 9.200 | 6.400 | wall | 0 | 257 | 7.236 | 2.753 | 28.831 |
| indoor_2 | 7 | 1.200 | 6.400 | -2.732 (-0.870*pi) | 7.200 | 5.000 | wall | 0 | 1338 | 5.499 | 16.152 | 278.972 |
| indoor_2 | 8 | 7.000 | 2.800 | -2.274 (-3*pi/4) | 2.800 | 5.800 | timeout | 0 | 2050 | 5.949 | 20.459 | 460.009 |
| indoor_2 | 9 | 3.200 | 8.200 | 1.642 (pi/2) | 7.000 | 2.600 | timeout | 0 | 2050 | 5.919 | 27.169 | 247.176 |
| indoor_2 | 10 | 5.000 | 6.200 | -0.508 (-0.162*pi) | 9.200 | 0.800 | timeout | 0 | 2050 | 6.627 | 1.696 | 218.350 |
