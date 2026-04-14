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
- selected test map: `indoor_3`
- repeats na mapu: `3`
- timeout: `2050` kroku (`205.0 s` pri `Ts = 0.1 s`)
- mode cases: `regenerate`
- start/cil/theta: nahodne generovane pro kazdy beh nebo znovu pouzite z CSV podle mode
- validace pripadu: clearance + existence cesty pres planner
- cases file: `project_test_all_maps_cases_test_indoor_3.csv`

- benchmark byl ukoncen predcasne na prvnim failu: `wall`, mapa `indoor_3`, repeat `3`

## Konfigurace behu

| Mapa | Priklad start x [m] | Priklad start y [m] | Priklad start theta [rad] | Priklad cil x [m] | Priklad cil y [m] | Max vel [m/s] |
|---|---:|---:|---:|---:|---:|---:|
| indoor_3 | 0.800 | 3.800 | -2.882 (-0.917*pi) | 6.400 | 5.000 | 1.0 |
## Souhrn

| Mapa | Success rate | Goal | Wall | Out | Timeout | Mean steps (success) | Mean goal error [m] | Mean runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| indoor_3 | 0.67 | 2 | 1 | 0 | 0 | 1365.5 | 2.630 | 63.322 |

## Detailni behy

| Mapa | Repeat | Start x [m] | Start y [m] | Start theta [rad] | Cil x [m] | Cil y [m] | Status | Success | Steps | Goal error [m] | Travelled [m] | Runtime [s] |
|---|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|
| indoor_3 | 1 | 0.800 | 3.800 | -2.882 (-0.917*pi) | 6.400 | 5.000 | goal | 1 | 1342 | 0.492 | 21.931 | 83.471 |
| indoor_3 | 2 | 9.200 | 3.000 | -2.626 (-0.836*pi) | 3.400 | 9.200 | goal | 1 | 1389 | 0.494 | 29.798 | 76.968 |
| indoor_3 | 3 | 2.400 | 9.200 | 1.642 (pi/2) | 8.800 | 6.400 | wall | 0 | 543 | 6.904 | 7.486 | 29.526 |
