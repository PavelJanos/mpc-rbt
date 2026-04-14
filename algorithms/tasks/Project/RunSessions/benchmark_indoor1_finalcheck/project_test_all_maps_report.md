# Project - Benchmark jedne mapy

Benchmark pouziva stejnou hlavni smycku jako `main`, ale bez GUI a bez `waitforbuttonpress`.

Pouzite nastaveni:
- planner: `A*`
- smoothing: `chaikin`
- controller: `pure pursuit`
- max velocity: `1.0 m/s`
- benchmark mode: `test`
- view mode: `true`
- stop mode: `first_failure`
- selected test map: `indoor_1`
- repeats na mapu: `10`
- timeout: `2050` kroku (`205.0 s` pri `Ts = 0.1 s`)
- mode cases: `regenerate`
- start/cil/theta: nahodne generovane pro kazdy beh nebo znovu pouzite z CSV podle mode
- validace pripadu: clearance + existence cesty pres planner
- cases file: `project_test_all_maps_cases_test_indoor_1.csv`

## Konfigurace behu

| Mapa | Priklad start x [m] | Priklad start y [m] | Priklad start theta [rad] | Priklad cil x [m] | Priklad cil y [m] | Max vel [m/s] |
|---|---:|---:|---:|---:|---:|---:|
| indoor_1 | 9.200 | 8.400 | -2.882 (-0.917*pi) | 0.800 | 0.800 | 1.0 |
## Souhrn

| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_1 | 1.00 | 10 | 0 | 0 | 0 | 960.8 | 0.491 | 46.479 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_1 | 1 | 9.200 | 8.400 | -2.882 (-0.917*pi) | 0.800 | 0.800 | goal | 1 | 1158 | 0.487 | 22.488 | 67.175 |
| indoor_1 | 2 | 2.000 | 0.800 | -0.457 (-0.145*pi) | 9.200 | 9.200 | goal | 1 | 1497 | 0.491 | 24.782 | 71.347 |
| indoor_1 | 3 | 8.000 | 6.000 | 1.281 (0.408*pi) | 2.600 | 3.000 | goal | 1 | 1261 | 0.493 | 23.801 | 65.538 |
| indoor_1 | 4 | 0.800 | 3.800 | 2.957 (0.941*pi) | 8.200 | 7.200 | goal | 1 | 816 | 0.487 | 17.769 | 37.320 |
| indoor_1 | 5 | 9.200 | 3.600 | -0.456 (-0.145*pi) | 4.800 | 9.200 | goal | 1 | 504 | 0.493 | 11.303 | 22.747 |
| indoor_1 | 6 | 5.200 | 0.800 | -2.486 (-0.791*pi) | 2.400 | 5.600 | goal | 1 | 523 | 0.492 | 10.183 | 24.963 |
| indoor_1 | 7 | 2.600 | 4.600 | -3.085 (-pi) | 8.000 | 5.000 | goal | 1 | 1020 | 0.493 | 20.341 | 46.848 |
| indoor_1 | 8 | 0.800 | 9.200 | 2.539 (0.808*pi) | 9.200 | 0.800 | goal | 1 | 587 | 0.492 | 13.329 | 24.644 |
| indoor_1 | 9 | 3.600 | 8.800 | 1.418 (0.451*pi) | 7.400 | 2.200 | goal | 1 | 386 | 0.488 | 9.221 | 18.288 |
| indoor_1 | 10 | 6.200 | 3.600 | 2.155 (2*pi/3) | 0.800 | 9.200 | goal | 1 | 1856 | 0.499 | 30.444 | 85.922 |
